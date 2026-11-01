"""Reconstruction of the published A formulas, not the missing frozen source."""
from dataclasses import asdict, dataclass
from typing import ClassVar
import numpy as np

A_VERSION = "A-RECONSTRUCTED-1.0.0"


@dataclass(frozen=True)
class APFParameters:
    k_att: float = 1.
    k_obstacle: float = .12
    k_robot: float = .18
    d0_obstacle: float = 1.4
    d0_robot: float = .9
    softening: float = .1
    max_repulsion: float = 12.
    force_epsilon: float = 1e-10

    def __post_init__(self):
        for key, value in asdict(self).items():
            if not np.isfinite(value) or value < 0:
                raise ValueError(key)
        if min(self.d0_obstacle, self.d0_robot, self.softening, self.max_repulsion) <= 0:
            raise ValueError("distances and cap must be positive")
        if self.softening >= min(self.d0_obstacle, self.d0_robot):
            raise ValueError("softening must be below influence gaps")


@dataclass(frozen=True)
class APFForces:
    attraction: np.ndarray
    obstacles: np.ndarray
    robots: np.ndarray
    total: np.ndarray


@dataclass(frozen=True)
class APFController:
    params: APFParameters = APFParameters()
    version: ClassVar[str] = A_VERSION

    def metadata(self):
        return dict(version=self.version, parameters=asdict(self.params),
                    provenance="reconstructed_from_BASELINE_A")

    def forces(self, obs):
        pr = self.params
        attraction = pr.k_att * (np.asarray(obs.goal) - obs.self_position)
        obstacles, robots = np.zeros(2), np.zeros(2)
        def repel(z, radius, gain, cutoff, fallback):
            d = float(np.linalg.norm(z))
            gap = d - radius
            if gap >= cutoff or gain == 0:
                return np.zeros(2)
            h = max(gap, pr.softening)
            mag = min(pr.max_repulsion, gain*(1/h-1/cutoff)/h**2)
            normal = -np.asarray(z)/d if d > 1e-12 else np.array([fallback, 0.])
            return mag * normal
        for circle in obs.obstacles:
            obstacles += repel(circle.relative_center, obs.robot_radius+circle.radius,
                               pr.k_obstacle, pr.d0_obstacle, 1.)
        for neighbor in obs.neighbors:
            robots += repel(neighbor.relative_position, 2*obs.robot_radius,
                            pr.k_robot, pr.d0_robot,
                            1. if obs.robot_id > neighbor.robot_id else -1.)
        return APFForces(attraction, obstacles, robots, attraction+obstacles+robots)

    def direction(self, obs):
        force = self.forces(obs).total
        norm = float(np.linalg.norm(force))
        return force/norm if norm > self.params.force_epsilon else np.zeros(2)

    def command(self, obs):
        distance = float(np.linalg.norm(np.asarray(obs.goal)-obs.self_position))
        return self.direction(obs) * min(obs.max_speed, distance/obs.dt)
