"""Stateless single-agent controller and local trajectory stagnation check."""

from dataclasses import dataclass
import numpy as np
from .fields import total_force
from .geometry import point2


@dataclass(frozen=True)
class VortexController:
    k_att: float = 1.0
    k_rep: float = 2.0
    d0: float = 2.0
    k_circ: float = 0.5

    def direction(self, position, goal, obstacles) -> np.ndarray:
        force = total_force(position, goal, obstacles,
                            self.k_att, self.k_rep, self.d0, self.k_circ)
        magnitude = float(np.linalg.norm(force))
        if not np.isfinite(magnitude):
            raise FloatingPointError("Nonfinite force")
        # No unsafe fallback towards the goal when forces cancel.
        return force / magnitude if magnitude > 1e-10 else np.zeros(2)


def is_stagnant(recent_positions, displacement_limit: float = 0.18,
                 ratio_limit: float = 0.2) -> bool:
    """Detect a stop or closed/oscillating trajectory in a FULL window.

    The caller controls window length. Slow straight motion has ratio 1,
    regardless of step size, and is never classified as stagnation.
    """
    pts = np.asarray(recent_positions, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
        raise ValueError("Need at least two 2D points")
    if not np.all(np.isfinite(pts)):
        raise ValueError("Nonfinite trajectory")
    if not (np.isfinite(displacement_limit) and displacement_limit > 0
            and np.isfinite(ratio_limit) and 0 < ratio_limit < 1):
        raise ValueError("Invalid stagnation thresholds")
    distance = float(np.linalg.norm(pts[-1] - pts[0]))
    travelled = float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())
    return distance < displacement_limit and (
        travelled <= 1e-12 or distance / travelled < ratio_limit
    )
