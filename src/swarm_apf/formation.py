"""Reconstructed B geometry and local controller; common open-loop reference."""
from dataclasses import asdict, dataclass
import math
import numpy as np
from .models import vec2

B_VERSION = "B-FORMATION-RECONSTRUCTED-1.0.0"


@dataclass(frozen=True)
class FormationLayout:
    robot_ids: tuple[int, ...]
    kind: str = "line"
    spacing: float = 1.4

    def __post_init__(self):
        ids = tuple(self.robot_ids)
        if not ids or len(set(ids)) != len(ids) or any(type(i) is not int for i in ids):
            raise ValueError("unique integer IDs required")
        if self.kind not in ("line", "V") or not math.isfinite(self.spacing) or self.spacing <= 0:
            raise ValueError("invalid layout")
        object.__setattr__(self, "robot_ids", ids)


@dataclass(frozen=True)
class FormationSignal:
    anchor: tuple[float, float]
    heading: tuple[float, float]


@dataclass(frozen=True)
class FormationPlan:
    start_anchor: tuple[float, float]
    group_goal: tuple[float, float]
    cruise_speed: float = .5
    heading: tuple[float, float] | None = None

    def __post_init__(self):
        start, goal = vec2(self.start_anchor), vec2(self.group_goal)
        if not math.isfinite(self.cruise_speed) or self.cruise_speed <= 0:
            raise ValueError("positive cruise_speed required")
        delta = np.asarray(goal)-start
        heading = vec2(self.heading) if self.heading is not None else (
            vec2(delta) if np.linalg.norm(delta) > 0 else (1., 0.))
        if np.linalg.norm(heading) == 0:
            raise ValueError("nonzero heading required")
        object.__setattr__(self, "start_anchor", start)
        object.__setattr__(self, "group_goal", goal)
        object.__setattr__(self, "heading", vec2(np.asarray(heading)/np.linalg.norm(heading)))

    @property
    def arrival_time(self):
        return float(np.linalg.norm(np.asarray(self.group_goal)-self.start_anchor))/self.cruise_speed

    def at(self, t):
        if not math.isfinite(t) or t < 0:
            raise ValueError("nonnegative finite time required")
        fraction = min(t/self.arrival_time, 1.) if self.arrival_time else 1.
        anchor = np.asarray(self.start_anchor)+fraction*(np.asarray(self.group_goal)-self.start_anchor)
        return FormationSignal(vec2(anchor), self.heading)


def slot_position(layout, signal, robot_id):
    if robot_id not in layout.robot_ids:
        raise ValueError("unknown slot ID")
    k = layout.robot_ids.index(robot_id)
    if layout.kind == "line":
        forward, left = 0., (k-(len(layout.robot_ids)-1)/2)*layout.spacing
    elif k == 0:
        forward, left = 0., 0.
    else:
        level = (k+1)//2
        forward, left = -level*layout.spacing, level*layout.spacing*(1 if k%2 else -1)
    e = np.asarray(signal.heading)
    e = e/np.linalg.norm(e)
    return np.asarray(signal.anchor)+forward*e+left*np.array([-e[1], e[0]])


@dataclass(frozen=True)
class FormationParameters:
    w_formation: float = 1.
    k_obstacle: float = .12
    k_robot: float = .18
    d0_obstacle: float = 1.4
    d0_robot: float = .9
    interaction_radius: float = 2.5
    softening: float = .1
    max_repulsion: float = 12.
    force_epsilon: float = 1e-10

    def __post_init__(self):
        for key, value in asdict(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(key)
        if min(self.d0_obstacle, self.d0_robot, self.interaction_radius,
               self.softening, self.max_repulsion) <= 0:
            raise ValueError("positive distances and cap required")
        if self.softening >= min(self.d0_obstacle, self.d0_robot):
            raise ValueError("invalid softening")


@dataclass(frozen=True)
class FormationController:
    layout: FormationLayout
    plan: FormationPlan
    robot_id: int
    params: FormationParameters = FormationParameters()
    version = B_VERSION

    def __post_init__(self):
        if self.robot_id not in self.layout.robot_ids:
            raise ValueError("unknown controller ID")

    def metadata(self):
        return dict(version=self.version, parameters=asdict(self.params),
                    layout=asdict(self.layout), plan=asdict(self.plan),
                    provenance="reconstructed_from_BASELINE_B")

    def forces_at(self, obs, t):
        if obs.robot_id != self.robot_id:
            raise ValueError("observation belongs to another robot")
        pr = self.params
        total = pr.w_formation*(slot_position(self.layout, self.plan.at(t), self.robot_id)
                                - np.asarray(obs.self_position))
        def add(z, radius, gain, cutoff, fallback):
            d = float(np.linalg.norm(z))
            gap = d-radius
            if gap >= cutoff or gain == 0:
                return np.zeros(2)
            h = max(gap, pr.softening)
            magnitude = min(pr.max_repulsion, gain*(1/h-1/cutoff)/h**2)
            return magnitude*(-np.asarray(z)/d if d > 1e-12 else np.array([fallback, 0.]))
        for circle in obs.obstacles:
            total += add(circle.relative_center, obs.robot_radius+circle.radius,
                         pr.k_obstacle, pr.d0_obstacle, 1.)
        for nb in obs.neighbors:
            if np.linalg.norm(nb.relative_position) <= pr.interaction_radius:
                total += add(nb.relative_position, 2*obs.robot_radius, pr.k_robot, pr.d0_robot,
                             1. if self.robot_id > nb.robot_id else -1.)
        return total

    def command_at(self, obs, t):
        total = self.forces_at(obs, t)
        norm = float(np.linalg.norm(total))
        return total/norm*min(obs.max_speed, norm/obs.dt) if norm > self.params.force_epsilon else np.zeros(2)

    def command(self, obs):
        return self.command_at(obs, obs.time)


class FormationTeam:
    """Dispatch local observations by fixed ID; no world state is accepted."""
    def __init__(self, layout, plan, params=FormationParameters()):
        self.controllers = {i: FormationController(layout, plan, i, params) for i in layout.robot_ids}

    def command(self, obs):
        return self.controllers[obs.robot_id].command(obs)

    def metadata(self):
        return next(iter(self.controllers.values())).metadata()
