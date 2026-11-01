"""Local command contract; the simulator owns collision checking."""
from typing import Protocol
import numpy as np
from .models import Observation


class SwarmController(Protocol):
    def command(self, observation: Observation) -> np.ndarray: ...


class GoalController:
    """Straight-line diagnostic controller, without collision avoidance."""
    def command(self, observation: Observation) -> np.ndarray:
        delta = np.asarray(observation.goal) - observation.self_position
        distance = float(np.linalg.norm(delta))
        if distance == 0:
            return np.zeros(2)
        return delta / distance * min(observation.max_speed, distance / observation.dt)
