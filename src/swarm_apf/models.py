"""E1 contracts: immutable snapshots and strictly local observations."""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


def _positive(name: str, value: float, *, allow_zero: bool = False) -> None:
    if not math.isfinite(value) or (value < 0 if allow_zero else value <= 0):
        raise ValueError(f"{name} must be finite and {'nonnegative' if allow_zero else 'positive'}")


def coordinates(values, *, label: str = "positions") -> np.ndarray:
    """Validate an N-by-2 array and return a new, independent array."""
    arr = np.array(values, dtype=float, copy=True)
    if arr.ndim != 2 or arr.shape[1] != 2 or len(arr) < 1 or not np.all(np.isfinite(arr)):
        raise ValueError(f"{label} must have finite shape (N, 2), N >= 1")
    return arr


def vec2(values, *, label: str = "vector") -> tuple[float, float]:
    arr = np.asarray(values, dtype=float)
    if arr.shape != (2,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{label} must contain two finite numbers")
    return (float(arr[0]), float(arr[1]))


@dataclass(frozen=True)
class SwarmConfig:
    dt: float = 0.05
    robot_radius: float = 0.20
    max_speed: float = 0.70
    neighbor_radius: float = 2.50           # centre-to-centre distance
    obstacle_sensor_radius: float = 3.00   # clearance from robot surface
    goal_tolerance: float = 0.25
    epsilon_geom: float = 1e-9

    def __post_init__(self) -> None:
        for key in ("dt", "robot_radius", "max_speed", "neighbor_radius",
                    "obstacle_sensor_radius", "goal_tolerance"):
            _positive(key, getattr(self, key))
        _positive("epsilon_geom", self.epsilon_geom, allow_zero=True)


@dataclass(frozen=True)
class WorldState:
    """Full simulator-only snapshot; arrays are owned copies and read-only."""
    robot_ids: tuple[int, ...]
    positions: np.ndarray
    velocities: np.ndarray
    time: float = 0.0

    def __post_init__(self) -> None:
        ids = tuple(self.robot_ids)
        if any(isinstance(i, bool) or not isinstance(i, (int, np.integer)) for i in ids):
            raise ValueError("robot_ids must be integers")
        if len(set(ids)) != len(ids) or not ids:
            raise ValueError("robot_ids must be nonempty and unique")
        p = coordinates(self.positions)
        v = coordinates(self.velocities, label="velocities")
        if p.shape != v.shape or len(p) != len(ids):
            raise ValueError("positions, velocities, and robot_ids must have the same N")
        if not math.isfinite(self.time) or self.time < 0:
            raise ValueError("time must be finite and nonnegative")
        # The backing store is immutable bytes: callers cannot flip WRITEABLE
        # back to True and alter a snapshot behind another robot's back.
        p = np.frombuffer(p.tobytes(), dtype=float).reshape(p.shape)
        v = np.frombuffer(v.tobytes(), dtype=float).reshape(v.shape)
        object.__setattr__(self, "robot_ids", ids)
        object.__setattr__(self, "positions", p)
        object.__setattr__(self, "velocities", v)


# Tuple coordinates, rather than mutable numpy arrays, give immutable local contracts.
@dataclass(frozen=True)
class NeighborObservation:
    robot_id: int
    relative_position: tuple[float, float]
    relative_velocity: tuple[float, float]


@dataclass(frozen=True)
class CircleObservation:
    relative_center: tuple[float, float]
    radius: float


@dataclass(frozen=True)
class Observation:
    time: float
    dt: float
    robot_id: int
    self_position: tuple[float, float]
    self_velocity: tuple[float, float]
    goal: tuple[float, float]               # individual target / assigned slot
    max_speed: float                       # publicly known physical limit
    neighbors: tuple[NeighborObservation, ...]
    obstacles: tuple[CircleObservation, ...]
    robot_radius: float = 0.20           # known physical parameter
