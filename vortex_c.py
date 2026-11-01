"""Baseline C: local formation APF plus fixed-sign obstacle circulation.

Pure per-agent controller for the documented Formation B observation/layout/plan
contract. Does not import the simulator or its global state. The existing B
source is deliberately not overwritten or reproduced in this package.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Callable, ClassVar
import math
import numpy as np

C_VERSION = "C-FORMATION-VORTEX-1.0.0"
_EPS = 1e-12


def _vec(v: object, label: str) -> np.ndarray:
    out = np.asarray(v, dtype=float)
    if out.shape != (2,) or not np.all(np.isfinite(out)):
        raise ValueError(f"{label} must be a finite 2-vector")
    return out


def _attr(obj: object, *names: str) -> object:
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    raise AttributeError(f"{type(obj).__name__} requires one of {names}")


def _positive(v: object, name: str, *, allow_zero: bool = False) -> float:
    try:
        x = float(v)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(x) or (x < 0 if allow_zero else x <= 0):
        raise ValueError(f"{name} must be finite and {'nonnegative' if allow_zero else 'positive'}")
    return x


def _repel(gap: float, gain: float, d0: float, softening: float, cap: float) -> float:
    """Bounded B radial magnitude: gap is surface-to-surface distance."""
    if gap >= d0 or gain == 0:
        return 0.0
    g = max(gap, softening)
    return min(cap, gain * (1.0 / g - 1.0 / d0) / (g * g))


def _outward(relative_center: np.ndarray, distance: float) -> np.ndarray:
    """Outward normal (obstacle/neighbor center -> current agent)."""
    return -relative_center / distance if distance > _EPS else np.array([1.0, 0.0])


def slot_from_b_contract(layout: object, signal: object, robot_id: object) -> np.ndarray:
    """Read the published B layout/signal interface, without accessing WorldState.

    For integration, verify against B.slot_position in original repo and use
    that function via a thin adapter if its real attribute names differ.
    """
    ids = tuple(_attr(layout, "robot_ids"))
    if not ids or len(set(ids)) != len(ids):
        raise ValueError("layout.robot_ids must be nonempty and unique")
    if robot_id not in ids:
        raise ValueError(f"robot_id {robot_id!r} has no preassigned formation slot")
    spacing = _positive(_attr(layout, "spacing"), "spacing")
    kind = _attr(layout, "kind")
    k = ids.index(robot_id)
    if kind == "line":
        forward, left = 0.0, (k - (len(ids) - 1) / 2.0) * spacing
    elif kind == "V":
        if k == 0:
            forward, left = 0.0, 0.0
        else:
            level = (k + 1) // 2
            forward = -level * spacing
            left = level * spacing * (1.0 if k % 2 else -1.0)
    else:
        raise ValueError("kind must be 'line' or 'V'")
    anchor = _vec(_attr(signal, "anchor"), "signal.anchor")
    heading = _vec(_attr(signal, "heading"), "signal.heading")
    norm = float(np.linalg.norm(heading))
    if norm <= _EPS:
        raise ValueError("signal.heading must be nonzero")
    e = heading / norm
    n = np.array([-e[1], e[0]])
    return anchor + forward * e + left * n


@dataclass(frozen=True)
class VortexCParameters:
    # Documented B defaults: do not tune B silently to make C look better.
    w_formation: float = 1.0
    k_obstacle: float = 0.12
    k_robot: float = 0.18
    d0_obstacle: float = 1.4
    d0_robot: float = 0.9
    interaction_radius: float = 2.5
    softening: float = 0.10
    max_repulsion: float = 12.0
    force_epsilon: float = 1e-10
    # New in C, dimensionless tangential-to-radial gain and fixed global sign.
    k_circ: float = 0.5
    circulation_sign: int = +1  # +1 CCW (original source), -1 CW

    def __post_init__(self) -> None:
        for field in ("w_formation", "k_obstacle", "k_robot", "k_circ", "force_epsilon"):
            _positive(getattr(self, field), field, allow_zero=True)
        for field in ("d0_obstacle", "d0_robot", "interaction_radius", "softening", "max_repulsion"):
            _positive(getattr(self, field), field)
        if self.softening >= min(self.d0_obstacle, self.d0_robot):
            raise ValueError("softening must be smaller than both influence gaps")
        if type(self.circulation_sign) is not int or self.circulation_sign not in (-1, +1):
            raise ValueError("circulation_sign must be integer +1 or -1")


@dataclass(frozen=True)
class VortexCForces:
    formation: np.ndarray
    obstacles_radial: np.ndarray
    obstacles_tangential: np.ndarray
    robots_radial: np.ndarray
    total: np.ndarray


@dataclass(frozen=True)
class FormationVortexController:
    layout: object
    plan: object
    robot_id: object
    params: VortexCParameters = VortexCParameters()
    slot_provider: Callable[[object, object, object], np.ndarray] | None = None
    version: ClassVar[str] = C_VERSION

    def __post_init__(self) -> None:
        ids = tuple(_attr(self.layout, "robot_ids"))
        if self.robot_id not in ids or len(set(ids)) != len(ids):
            raise ValueError("controller ID must be a unique ID assigned by layout")

    def metadata(self) -> dict:
        return {"stage": "C", "version": self.version, "parameters": asdict(self.params),
                "circulation_policy": "fixed-ccw" if self.params.circulation_sign == 1 else "fixed-cw",
                "uses_shared_reference": True,
                "slot_source": "B.slot_position" if self.slot_provider is not None else "documented_B_layout_proxy"}

    def slot_at(self, t: float) -> np.ndarray:
        _positive(t, "time", allow_zero=True)
        signal = self.plan.at(t)
        resolver = self.slot_provider or slot_from_b_contract
        return _vec(resolver(self.layout, signal, self.robot_id), "formation_slot")

    def forces_at(self, observation: object, t: float) -> VortexCForces:
        p = _vec(_attr(observation, "self_position"), "self_position")
        r = _positive(_attr(observation, "robot_radius"), "robot_radius", allow_zero=True)
        pr = self.params
        formation = pr.w_formation * (self.slot_at(t) - p)
        radial = np.zeros(2)
        tangential = np.zeros(2)
        robots = np.zeros(2)
        # Only sensor-provided circles; no world/map access.
        for obstacle in _attr(observation, "obstacles"):
            z = _vec(_attr(obstacle, "relative_center"), "obstacle.relative_center")
            radius = _positive(_attr(obstacle, "radius"), "obstacle.radius")
            d = float(np.linalg.norm(z))
            gap = d - r - radius
            mag = _repel(gap, pr.k_obstacle, pr.d0_obstacle, pr.softening, pr.max_repulsion)
            if mag == 0:
                continue
            n = _outward(z, d)
            radial += mag * n
            # Match source qualitative contact policy: inside/on surface =>
            # radial evacuation only. No RNG or undefined tangent at center.
            if gap > 0:
                tangential += (pr.circulation_sign * pr.k_circ * mag
                               * np.array([-n[1], n[0]]))
        for neighbor in _attr(observation, "neighbors"):
            z = _vec(_attr(neighbor, "relative_position"), "neighbor.relative_position")
            d = float(np.linalg.norm(z))
            if d > pr.interaction_radius:
                continue
            gap = d - 2 * r
            mag = _repel(gap, pr.k_robot, pr.d0_robot, pr.softening, pr.max_repulsion)
            if mag == 0:
                continue
            if d > _EPS:
                n = -z / d
            else:
                other_id = _attr(neighbor, "robot_id", "id")
                if other_id == self.robot_id:
                    raise ValueError("two coincident robots cannot have the same ID")
                n = np.array([1.0 if str(self.robot_id) > str(other_id) else -1.0, 0.0])
            robots += mag * n
        total = formation + radial + tangential + robots
        if not np.isfinite(total).all():
            raise ValueError("nonfinite virtual force")
        return VortexCForces(formation, radial, tangential, robots, total)

    def forces(self, observation: object) -> VortexCForces:
        return self.forces_at(observation, float(_attr(observation, "time", "t")))

    def direction(self, observation: object) -> np.ndarray:
        total = self.forces(observation).total
        magnitude = float(np.linalg.norm(total))
        return total / magnitude if magnitude > self.params.force_epsilon else np.zeros(2)

    def command_at(self, observation: object, t: float) -> np.ndarray:
        dt = _positive(_attr(observation, "dt"), "dt")
        vmax = _positive(_attr(observation, "max_speed"), "max_speed")
        total = self.forces_at(observation, t).total
        magnitude = float(np.linalg.norm(total))
        if magnitude <= self.params.force_epsilon:
            return np.zeros(2)
        # Preserves B conversion: forces are virtual steering, NOT Newtons.
        u = total / magnitude * min(vmax, magnitude / dt)
        if not np.isfinite(u).all():
            raise ValueError("nonfinite command")
        return u

    def command(self, observation: object) -> np.ndarray:
        return self.command_at(observation, float(_attr(observation, "time", "t")))
