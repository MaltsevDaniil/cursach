"""Valid 2D circles and continuous segment–circle clearance."""

from dataclasses import dataclass
import math
import numpy as np

EPS_GEOM = 1e-9


def point2(value) -> np.ndarray:
    """Return a validated independent float vector of shape (2,)."""
    p = np.asarray(value, dtype=float)
    if p.shape != (2,) or not np.all(np.isfinite(p)):
        raise ValueError("Expected two finite coordinates")
    return p.copy()


@dataclass(frozen=True)
class Obstacle:
    center: tuple[float, float]
    radius: float

    def __post_init__(self) -> None:
        c = point2(self.center)
        r = float(self.radius)
        if not math.isfinite(r) or r <= 0:
            raise ValueError("Obstacle radius must be positive and finite")
        object.__setattr__(self, "center", (float(c[0]), float(c[1])))
        object.__setattr__(self, "radius", r)

    def distance(self, point, agent_radius: float = 0.0) -> float:
        """Signed surface clearance; 0 means touching."""
        _check_agent_radius(agent_radius)
        return float(np.linalg.norm(point2(point) - self.center) - self.radius - agent_radius)

    def contains(self, point, agent_radius: float = 0.0) -> bool:
        return self.distance(point, agent_radius) <= 0.0


def _check_agent_radius(value: float) -> None:
    if not math.isfinite(value) or value < 0:
        raise ValueError("agent_radius must be nonnegative and finite")


def segment_circle_clearance(a, b, obstacle: Obstacle,
                             agent_radius: float = 0.0) -> float:
    """Signed minimum clearance of the ENTIRE closed segment a→b."""
    _check_agent_radius(agent_radius)
    start, end = point2(a), point2(b)
    delta = end - start
    length_squared = float(np.dot(delta, delta))
    if length_squared == 0.0:
        closest = start
    else:
        tau = float(np.clip(np.dot(np.asarray(obstacle.center) - start, delta)
                            / length_squared, 0.0, 1.0))
        closest = start + tau * delta
    return float(np.linalg.norm(closest - obstacle.center)
                 - obstacle.radius - agent_radius)


def segment_hits_obstacle(a, b, obstacle: Obstacle,
                          agent_radius: float = 0.0,
                          epsilon: float = EPS_GEOM) -> bool:
    if not math.isfinite(epsilon) or epsilon < 0:
        raise ValueError("epsilon must be nonnegative and finite")
    return segment_circle_clearance(a, b, obstacle, agent_radius) <= epsilon
