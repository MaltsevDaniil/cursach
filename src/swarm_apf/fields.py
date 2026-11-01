"""Original APF/vortex composition with bounded near-surface behavior."""

import math
import numpy as np
from .geometry import Obstacle, point2


def _parameters(k_att=None, k_rep=None, d0=None, k_circ=None,
                softening=None, max_repulsion=None) -> None:
    for name, value in (("k_att", k_att), ("k_rep", k_rep),
                        ("d0", d0), ("k_circ", k_circ),
                        ("softening", softening),
                        ("max_repulsion", max_repulsion)):
        if value is not None and not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    if k_att is not None and k_att < 0:
        raise ValueError("k_att must be >= 0")
    if k_rep is not None and k_rep < 0:
        raise ValueError("k_rep must be >= 0")
    for name, value in (("d0", d0), ("softening", softening),
                        ("max_repulsion", max_repulsion)):
        if value is not None and value <= 0:
            raise ValueError(f"{name} must be > 0")


def attractive_force(position, goal, k_att: float = 1.0) -> np.ndarray:
    _parameters(k_att=k_att)
    return k_att * (point2(goal) - point2(position))


def vortex_repulsive_force(position, obstacles: tuple[Obstacle, ...],
                           k_rep: float = 2.0, d0: float = 2.0,
                           k_circ: float = 0.5,
                           softening: float = 0.05,
                           max_repulsion: float = 10000.0) -> np.ndarray:
    """CCW circulation (+90°); old formula unchanged in its regular domain.

    For d < softening clamp the inverse-distance argument; also cap the
    radial amplitude. At the exact center select a deterministic normal.
    The same expression is used on either side of the circle boundary.
    """
    _parameters(k_rep=k_rep, d0=d0, k_circ=k_circ,
                softening=softening, max_repulsion=max_repulsion)
    p = point2(position)
    result = np.zeros(2, dtype=float)
    for obs in obstacles:
        vec = p - obs.center
        rho = float(np.linalg.norm(vec))
        d = rho - obs.radius
        if d >= d0:
            continue
        radial = vec / rho if rho > 0 else np.array([1.0, 0.0])
        safe_d = max(d, min(softening, 0.5 * d0))
        amplitude = k_rep * (1.0 / safe_d - 1.0 / d0) / (safe_d**2 + 1e-10)
        amplitude = min(amplitude, max_repulsion)
        tangent = np.array([-radial[1], radial[0]])
        result += amplitude * (radial + k_circ * tangent)
    return result


def total_force(position, goal, obstacles,
                k_att: float = 1.0, k_rep: float = 2.0,
                d0: float = 2.0, k_circ: float = 0.5) -> np.ndarray:
    return (attractive_force(position, goal, k_att)
            + vortex_repulsive_force(position, obstacles, k_rep, d0, k_circ))
