"""Single-agent Euler planner: decide → propose → check segment → commit."""

from dataclasses import dataclass
import math
import numpy as np
from .controller import VortexController, is_stagnant
from .geometry import EPS_GEOM, Obstacle, point2, segment_circle_clearance
from .metrics import calculate_path_length, minimum_path_clearance


@dataclass
class PathResult:
    status: str
    path: np.ndarray
    iterations: int  # attempted commands, including a rejected collision step
    path_length: float
    min_clearance: float
    collision_segment: np.ndarray | None = None

    @property
    def success(self) -> bool:
        return self.status == "SUCCESS"


def plan_path(start, goal, obstacles: tuple[Obstacle, ...] | list[Obstacle], *,
              step: float = 0.05, max_iterations: int = 5000,
              k_att: float = 1.0, k_rep: float = 2.0, d0: float = 2.0,
              k_circ: float = 0.5, goal_tolerance: float = 0.1,
              agent_radius: float = 0.0, stagnation_window: int = 40,
              stagnation_displacement: float = 0.18,
              stagnation_ratio: float = 0.2) -> PathResult:
    """Preserve original normalized direction; stop on collision, no repair.

    The legacy agent is a POINT (agent_radius=0). Nonzero radius is an
    optional clearance inflation, not a new multi-robot physics engine.
    """
    p, g = point2(start), point2(goal)
    obs = tuple(obstacles)
    if not (math.isfinite(step) and step > 0
            and math.isfinite(goal_tolerance) and goal_tolerance > 0
            and math.isfinite(agent_radius) and agent_radius >= 0):
        raise ValueError("Invalid step, tolerance or agent radius")
    if isinstance(max_iterations, bool) or not isinstance(max_iterations, int) or max_iterations < 0:
        raise ValueError("max_iterations must be a nonnegative integer")
    if isinstance(stagnation_window, bool) or not isinstance(stagnation_window, int) or stagnation_window < 2:
        raise ValueError("stagnation_window must be an integer >= 2")
    if not (math.isfinite(stagnation_displacement) and stagnation_displacement > 0
            and math.isfinite(stagnation_ratio) and 0 < stagnation_ratio < 1):
        raise ValueError("Invalid stagnation parameters")
    controller = VortexController(k_att, k_rep, d0, k_circ)
    # Validate coefficients even if the initial state needs zero steps.
    from .fields import _parameters
    _parameters(k_att=k_att, k_rep=k_rep, d0=d0, k_circ=k_circ)

    path = [p.copy()]
    clearance = minimum_path_clearance(np.asarray(path), obs, agent_radius)

    def finish(status, count, collision=None) -> PathResult:
        return PathResult(status, np.asarray(path), count,
                          calculate_path_length(np.asarray(path)),
                          clearance, collision)

    # Tangency is invalid even if a path starts exactly at the goal.
    if any(o.distance(p, agent_radius) <= EPS_GEOM or
           o.distance(g, agent_radius) <= EPS_GEOM for o in obs):
        return finish("INVALID_SCENE", 0)
    if np.linalg.norm(p - g) <= goal_tolerance:
        return finish("SUCCESS", 0)

    for i in range(1, max_iterations + 1):
        try:
            direction = controller.direction(p, g, obs)
            if not np.all(np.isfinite(direction)):
                return finish("NUMERIC_ERROR", i)
            if np.linalg.norm(direction) <= 1e-10:
                return finish("ZERO_FORCE", i)
            # Avoid the old free-space overshoot (D04); otherwise same step.
            actual_step = min(step, float(np.linalg.norm(g - p)))
            candidate = p + actual_step * direction
            if not np.all(np.isfinite(candidate)):
                return finish("NUMERIC_ERROR", i)
            segment_gap = min((segment_circle_clearance(p, candidate, o, agent_radius)
                               for o in obs), default=float("inf"))
        except (FloatingPointError, OverflowError):
            return finish("NUMERIC_ERROR", i)
        clearance = min(clearance, segment_gap)
        if segment_gap <= EPS_GEOM:
            return finish("COLLISION_OBSTACLE", i,
                          np.vstack([p, candidate]))
        p = candidate
        path.append(p.copy())
        if np.linalg.norm(p - g) <= goal_tolerance:
            return finish("SUCCESS", i)
        if len(path) > stagnation_window:
            if is_stagnant(path[-(stagnation_window + 1):],
                           stagnation_displacement, stagnation_ratio):
                return finish("STAGNATION", i)
    return finish("TIMEOUT", max_iterations)
