"""Experimental local Boids steering for the position-only E1 swarm simulator.

This module is independent of multi_apf.py (frozen Baseline A).  It only
consumes public per-robot observations; it does not access WorldState/maps.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import ClassVar
import math
import numpy as np

BOIDS_VERSION = "BOIDS-EXPERIMENT-1.0.0"
_EPS = 1e-12


def _vector(value: object, name: str) -> np.ndarray:
    v = np.asarray(value, dtype=float)
    if v.shape != (2,) or not np.isfinite(v).all():
        raise ValueError(f"{name} must be a finite 2-vector")
    return v


def _unit(v: np.ndarray) -> np.ndarray:
    mag = float(np.linalg.norm(v))
    return v / mag if mag > _EPS else np.zeros(2, dtype=float)


def _cap_unit(v: np.ndarray) -> np.ndarray:
    return v / max(1.0, float(np.linalg.norm(v)))


def _attr(obj: object, *names: str) -> object:
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    raise AttributeError(f"{type(obj).__name__} needs one of: {', '.join(names)}")


@dataclass(frozen=True)
class BoidsParameters:
    """Dimensionless steering weights and metric local interaction distances."""

    w_goal: float = 1.0
    w_separation: float = 2.0
    w_cohesion: float = 0.35
    w_alignment: float = 0.35
    w_obstacle: float = 0.0  # Explicit optional obstacle steering, OFF by default.
    separation_clearance: float = 0.75  # Surface-to-surface gap, NOT center distance.
    interaction_radius: float = 2.5    # Center-to-center, must fit E1 neighbor sensor.
    obstacle_clearance: float = 1.2    # Surface gap for optional obstacle steering.
    goal_stop_radius: float = 0.0      # Stop only at exact goal by default.

    def __post_init__(self) -> None:
        for key, value in asdict(self).items():
            if not math.isfinite(value):
                raise ValueError(f"{key} must be finite")
            if key.startswith("w_") or key == "goal_stop_radius":
                if value < 0:
                    raise ValueError(f"{key} must be >= 0")
            elif value <= 0:
                raise ValueError(f"{key} must be > 0")


@dataclass(frozen=True)
class BoidsTerms:
    """Local, dimensionless steering components; total already weighted."""

    goal: np.ndarray
    separation: np.ndarray
    cohesion: np.ndarray
    alignment: np.ndarray
    obstacle: np.ndarray
    total: np.ndarray


@dataclass(frozen=True)
class BoidsController:
    """Stateless E1-compatible controller: command(observation) -> (ux, uy)."""

    params: BoidsParameters = BoidsParameters()
    version: ClassVar[str] = BOIDS_VERSION

    def metadata(self) -> dict:
        return {"stage": "BOIDS_EXPERIMENT", "version": self.version,
                "parameters": asdict(self.params)}

    def forces(self, observation: object) -> BoidsTerms:
        p = _vector(_attr(observation, "self_position"), "self_position")
        g = _vector(_attr(observation, "goal", "goal_i"), "goal")
        r = float(_attr(observation, "robot_radius"))
        speed = float(_attr(observation, "max_speed"))
        if not math.isfinite(r) or r < 0 or not math.isfinite(speed) or speed <= 0:
            raise ValueError("robot_radius must be >= 0 and max_speed > 0")
        pr = self.params
        goal = _unit(g - p)
        sep_sum, coh_sum, align_sum = np.zeros(2), np.zeros(2), np.zeros(2)
        sep_count = near_count = 0
        self_v = None
        if pr.w_alignment > 0:
            self_v = _vector(_attr(observation, "self_velocity"), "self_velocity")

        for nb in _attr(observation, "neighbors"):
            relative = _vector(_attr(nb, "relative_position"), "relative_position")
            d = float(np.linalg.norm(relative))
            # The upstream sensor has already hidden out-of-range robots. An
            # additional local radius can only discard, never reveal, robots.
            if d > pr.interaction_radius:
                continue
            near_count += 1
            coh_sum += relative
            if self_v is not None:
                # E1 may expose absolute neighbor velocity or relative velocity.
                # Both are legitimate local measurements in one common frame.
                if hasattr(nb, "velocity"):
                    v_j = _vector(nb.velocity, "neighbor.velocity")
                else:
                    v_j = self_v + _vector(
                        _attr(nb, "relative_velocity"), "neighbor.relative_velocity")
                align_sum += v_j - self_v
            clearance = d - 2.0 * r
            if clearance < pr.separation_clearance:
                if d > _EPS:
                    away = -relative / d
                else:
                    # Invalid coincident disks: antisymmetric fallback if IDs
                    # exist. Without IDs, direction is physically undefined.
                    my_id = getattr(observation, "robot_id", getattr(observation, "self_id", None))
                    nb_id = getattr(nb, "robot_id", getattr(nb, "id", None))
                    if my_id is None or nb_id is None or my_id == nb_id:
                        away = np.zeros(2)
                    else:
                        away = np.array([1.0 if str(my_id) > str(nb_id) else -1.0, 0.0])
                factor = min(1.0, max(0.0, (pr.separation_clearance - clearance)
                                      / pr.separation_clearance))
                sep_sum += factor * away
                sep_count += 1

        sep = sep_sum / sep_count if sep_count else np.zeros(2)
        coh = _cap_unit(coh_sum / (near_count * pr.interaction_radius)) if near_count else np.zeros(2)
        align = _cap_unit(align_sum / (near_count * speed)) if near_count and self_v is not None else np.zeros(2)
        obs_sum = np.zeros(2)
        obs_count = 0
        if pr.w_obstacle > 0:
            for obstacle in _attr(observation, "obstacles"):
                relative = _vector(_attr(obstacle, "relative_center"), "relative_center")
                radius = float(_attr(obstacle, "radius"))
                if not math.isfinite(radius) or radius <= 0:
                    raise ValueError("obstacle radius must be positive and finite")
                distance = float(np.linalg.norm(relative))
                gap = distance - r - radius
                if gap < pr.obstacle_clearance:
                    away = -relative / distance if distance > _EPS else np.array([1.0, 0.0])
                    factor = min(1.0, max(0.0, (pr.obstacle_clearance - gap)
                                          / pr.obstacle_clearance))
                    obs_sum += factor * away
                    obs_count += 1
        obstacle_term = obs_sum / obs_count if obs_count else np.zeros(2)
        total = (pr.w_goal * goal + pr.w_separation * sep +
                 pr.w_cohesion * coh + pr.w_alignment * align +
                 pr.w_obstacle * obstacle_term)
        return BoidsTerms(goal, sep, coh, align, obstacle_term, total)

    def direction(self, observation: object) -> np.ndarray:
        return _unit(self.forces(observation).total)

    def command(self, observation: object) -> np.ndarray:
        p = _vector(_attr(observation, "self_position"), "self_position")
        g = _vector(_attr(observation, "goal", "goal_i"), "goal")
        distance = float(np.linalg.norm(g - p))
        dt = float(_attr(observation, "dt"))
        speed = float(_attr(observation, "max_speed"))
        if not math.isfinite(dt) or dt <= 0 or not math.isfinite(speed) or speed <= 0:
            raise ValueError("dt and max_speed must be positive and finite")
        if distance <= max(_EPS, self.params.goal_stop_radius):
            return np.zeros(2)
        return self.direction(observation) * min(speed, distance / dt)
