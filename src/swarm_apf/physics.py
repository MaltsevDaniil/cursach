"""Continuous collision checking in a synchronous, straight-line Euler step."""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from .geometry import Obstacle, segment_circle_clearance


@dataclass(frozen=True)
class CollisionEvent:
    kind: str                   # COLLISION_ROBOT or COLLISION_OBSTACLE
    robot_ids: tuple[int, ...]  # one ID for obstacle, two for robot pair
    obstacle_index: int | None
    clearance: float            # signed minimum over the entire proposed step
    fraction: float             # earliest contact in [0,1] of the proposed step
    time: float                 # time at earliest contact (not time of min distance)


@dataclass(frozen=True)
class SweepCheck:
    events: tuple[CollisionEvent, ...]
    min_robot_clearance: float
    min_obstacle_clearance: float


def _closest_fraction(a: np.ndarray, motion: np.ndarray) -> float:
    length2 = float(np.dot(motion, motion))
    return float(np.clip(-np.dot(a, motion) / length2, 0, 1)) if length2 > 0 else 0.0


def _first_contact_fraction(a: np.ndarray, motion: np.ndarray,
                            contact_radius: float, fallback: float) -> float:
    """Solve ||a + tau*motion|| == contact_radius, tau in [0,1]."""
    c = float(np.dot(a, a) - contact_radius**2)
    if c <= 0:
        return 0.0
    qa = float(np.dot(motion, motion))
    if qa == 0:
        return fallback
    qb = 2.0 * float(np.dot(a, motion))
    disc = qb * qb - 4.0 * qa * c
    if disc < 0:                 # near-tangent roundoff: closest point fallback
        return fallback
    root = (-qb - math.sqrt(disc)) / (2.0 * qa)
    return float(np.clip(root, 0.0, 1.0))


def moving_disk_pair_clearance(start_i, end_i, start_j, end_j,
                               radius_i: float, radius_j: float) -> float:
    """Min signed clearance for simultaneous linear motions, not endpoints."""
    if any(not math.isfinite(r) or r < 0 for r in (radius_i, radius_j)):
        raise ValueError("radii must be finite and nonnegative")
    si, ei, sj, ej = [np.asarray(x, dtype=float) for x in
                      (start_i, end_i, start_j, end_j)]
    if any(x.shape != (2,) or not np.all(np.isfinite(x)) for x in (si, ei, sj, ej)):
        raise ValueError("four finite 2D points required")
    rel = si - sj
    motion = (ei - si) - (ej - sj)
    tau = _closest_fraction(rel, motion)
    return float(np.linalg.norm(rel + tau * motion) - radius_i - radius_j)


def check_sweeps(start: np.ndarray, end: np.ndarray, robot_ids: tuple[int, ...],
                 obstacles: tuple[Obstacle, ...], robot_radius: float,
                 time: float, dt: float, epsilon: float = 1e-9) -> SweepCheck:
    """Check all pairs and ALL obstacles, even when outside sensing range."""
    events: list[CollisionEvent] = []
    min_robot, min_obstacle = float("inf"), float("inf")
    n = len(robot_ids)
    for i in range(n):
        for j in range(i + 1, n):
            rel = start[i] - start[j]
            motion = (end[i] - start[i]) - (end[j] - start[j])
            tau = _closest_fraction(rel, motion)
            gap = float(np.linalg.norm(rel + tau * motion) - 2 * robot_radius)
            min_robot = min(min_robot, gap)
            if gap <= epsilon:
                hit = _first_contact_fraction(rel, motion, 2 * robot_radius + epsilon, tau)
                events.append(CollisionEvent(
                    "COLLISION_ROBOT", (robot_ids[i], robot_ids[j]), None,
                    gap, hit, time + hit * dt))
        for k, obstacle in enumerate(obstacles):
            gap = segment_circle_clearance(start[i], end[i], obstacle, robot_radius)
            min_obstacle = min(min_obstacle, gap)
            if gap <= epsilon:
                rel = start[i] - np.asarray(obstacle.center)
                motion = end[i] - start[i]
                closest = _closest_fraction(rel, motion)
                hit = _first_contact_fraction(rel, motion,
                                              robot_radius + obstacle.radius + epsilon,
                                              closest)
                events.append(CollisionEvent(
                    "COLLISION_OBSTACLE", (robot_ids[i],), k,
                    gap, hit, time + hit * dt))
    # Deterministic earliest-contact status; robot first on exact timestamp tie.
    events.sort(key=lambda e: (e.fraction, 0 if e.kind == "COLLISION_ROBOT" else 1,
                               e.robot_ids, -1 if e.obstacle_index is None else e.obstacle_index))
    return SweepCheck(tuple(events), min_robot, min_obstacle)
