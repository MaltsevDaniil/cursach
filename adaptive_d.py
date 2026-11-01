"""Method D: local, stateful adaptive wrapper around *unmodified* baseline C.

One instance per robot per run. The state machine reads only local observations,
its own accepted positions over time, and a shared pre-agreed formation plan.
No WorldState, map, neighbor goals, or simulator callbacks are accessed here.
"""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, replace
from enum import Enum
from typing import Callable, ClassVar
import math
import numpy as np

from .vortex_c import FormationVortexController, VortexCParameters, _attr, _positive, _vec

D_VERSION = "D-ADAPTIVE-1.0.0"


class Mode(str, Enum):
    NORMAL = "NORMAL"
    PASSAGE = "PASSAGE"
    RECOVERY = "RECOVERY"


@dataclass(frozen=True)
class AdaptiveDParameters:
    # All clearances are *surface-to-surface* (not distances between centers).
    obstacle_enter: float = 0.65
    obstacle_exit: float = 1.00
    neighbor_enter: float = 0.40
    neighbor_exit: float = 0.70
    enter_samples: int = 2
    exit_samples: int = 5
    min_dwell: float = 0.60
    stall_window: float = 1.00
    stall_progress: float = 0.05
    recovery_progress: float = 0.20
    stall_samples: int = 3
    terminal_tolerance: float = 0.30
    formation_passage_fraction: float = 0.20
    formation_recovery_fraction: float = 0.10
    circulation_passage_multiplier: float = 2.0
    circulation_recovery_multiplier: float = 3.0

    def __post_init__(self) -> None:
        for name in ("obstacle_enter", "obstacle_exit", "neighbor_enter", "neighbor_exit",
                     "min_dwell", "stall_window", "stall_progress", "recovery_progress",
                     "terminal_tolerance", "formation_passage_fraction", "formation_recovery_fraction",
                     "circulation_passage_multiplier", "circulation_recovery_multiplier"):
            _positive(getattr(self, name), name, allow_zero=name in (
                "obstacle_enter", "neighbor_enter", "stall_progress", "terminal_tolerance",
                "formation_passage_fraction", "formation_recovery_fraction"))
        if not self.obstacle_enter < self.obstacle_exit or not self.neighbor_enter < self.neighbor_exit:
            raise ValueError("exit thresholds must exceed entry thresholds")
        if not 0 <= self.formation_recovery_fraction <= self.formation_passage_fraction <= 1:
            raise ValueError("formation fractions must satisfy 0<=recovery<=passage<=1")
        if self.recovery_progress <= self.stall_progress:
            raise ValueError("recovery_progress must exceed stall_progress")
        if self.circulation_passage_multiplier < 1 or self.circulation_recovery_multiplier < self.circulation_passage_multiplier:
            raise ValueError("circulation multipliers must be nondecreasing and >=1")
        for name in ("enter_samples", "exit_samples", "stall_samples"):
            value = getattr(self, name)
            if type(value) is not int or value < 1:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class DAblation:
    adapt_formation: bool = True
    use_vortex: bool = True
    restore_formation: bool = True


ABLATIONS: dict[str, DAblation] = {
    "full": DAblation(),
    "no_weight_adaptation": DAblation(adapt_formation=False),
    "no_vortex": DAblation(use_vortex=False),
    "no_formation_restoration": DAblation(restore_formation=False),
}


@dataclass(frozen=True)
class ModeRecord:
    robot_id: object
    time: float
    mode: str
    previous_mode: str
    transitioned: bool
    reason: str
    min_obstacle_gap: float | None
    min_neighbor_gap: float | None
    distance_to_final_slot: float
    progress_window: float | None
    crowded: bool
    stalled: bool
    w_formation: float
    k_circ: float
    circulation_sign: int
    command_x: float
    command_y: float
    # These are *proposed* commands, not simulator-accepted steps.

    def as_dict(self) -> dict:
        return asdict(self)


class AdaptiveDController:
    version: ClassVar[str] = D_VERSION

    def __init__(self, layout: object, plan: object, robot_id: object,
                 base_params: VortexCParameters | None = None,
                 adaptive_params: AdaptiveDParameters | None = None,
                 ablation: DAblation | None = None,
                 slot_provider: Callable[[object, object, object], np.ndarray] | None = None):
        self.base_params = base_params if base_params is not None else VortexCParameters()
        self.adaptive_params = adaptive_params if adaptive_params is not None else AdaptiveDParameters()
        self.ablation = ablation if ablation is not None else ABLATIONS["full"]
        if not isinstance(self.base_params, VortexCParameters):
            raise TypeError("base_params must be VortexCParameters")
        if not isinstance(self.adaptive_params, AdaptiveDParameters):
            raise TypeError("adaptive_params must be AdaptiveDParameters")
        if not isinstance(self.ablation, DAblation):
            raise TypeError("ablation must be DAblation")
        self.robot_id = robot_id
        self._base = FormationVortexController(layout, plan, robot_id, self.base_params, slot_provider)
        self._plan = plan
        self.reset()

    def reset(self) -> None:
        """Explicitly reset local memory before each run or replay."""
        self.mode = Mode.NORMAL
        self._mode_since: float | None = None
        self._last_time: float | None = None
        self._last_command: np.ndarray | None = None
        self._recent: deque[tuple[float, float]] = deque()
        self._enter_count = self._exit_count = self._stall_count = self._recover_count = 0
        self._ever_suppressed = False
        self.records: list[ModeRecord] = []

    def metadata(self) -> dict:
        return {"stage": "D", "version": self.version,
                "base_C": self._base.metadata(),
                "adaptive_parameters": asdict(self.adaptive_params),
                "ablation": asdict(self.ablation),
                "state_scope": "per-robot; reset per run",
                "progress_target": "fixed final slot from pre-shared B plan",
                "logs_are": "attempted controller commands; simulator tracks acceptance"}

    def _local_gaps(self, observation: object) -> tuple[float | None, float | None]:
        r = _positive(_attr(observation, "robot_radius"), "robot_radius", allow_zero=True)
        obs_gaps: list[float] = []
        nb_gaps: list[float] = []
        for obstacle in _attr(observation, "obstacles"):
            z = _vec(_attr(obstacle, "relative_center"), "relative_center")
            radius = _positive(_attr(obstacle, "radius"), "radius")
            obs_gaps.append(float(np.linalg.norm(z)) - r - radius)
        for neighbor in _attr(observation, "neighbors"):
            z = _vec(_attr(neighbor, "relative_position"), "relative_position")
            if float(np.linalg.norm(z)) <= self.base_params.interaction_radius:
                nb_gaps.append(float(np.linalg.norm(z)) - 2 * r)
        return (min(obs_gaps) if obs_gaps else None,
                min(nb_gaps) if nb_gaps else None)

    def _progress(self, t: float, distance: float) -> float | None:
        self._recent.append((t, distance))
        window = self.adaptive_params.stall_window
        # Keep the latest sample at or before t-window, if available.
        while len(self._recent) >= 2 and self._recent[1][0] <= t - window:
            self._recent.popleft()
        old_t, old_distance = self._recent[0]
        if t - old_t < 0.95 * window:
            return None
        return old_distance - distance

    def _switch(self, new_mode: Mode, t: float) -> None:
        self.mode = new_mode
        self._mode_since = t
        self._enter_count = self._exit_count = self._stall_count = self._recover_count = 0
        if new_mode is not Mode.NORMAL:
            self._ever_suppressed = True

    def _effective_coefficients(self) -> tuple[float, float]:
        ap, bp = self.adaptive_params, self.base_params
        if self.mode is Mode.PASSAGE:
            weight_fraction = ap.formation_passage_fraction
            vortex_multiplier = ap.circulation_passage_multiplier
        elif self.mode is Mode.RECOVERY:
            weight_fraction = ap.formation_recovery_fraction
            vortex_multiplier = ap.circulation_recovery_multiplier
        else:
            weight_fraction = 1.0
            vortex_multiplier = 1.0
            if self._ever_suppressed and not self.ablation.restore_formation:
                weight_fraction = ap.formation_passage_fraction
        weight = bp.w_formation * (weight_fraction if self.ablation.adapt_formation else 1.0)
        circulation = bp.k_circ * vortex_multiplier if self.ablation.use_vortex else 0.0
        return weight, circulation

    def command_at(self, observation: object, t: float) -> np.ndarray:
        t = _positive(t, "time", allow_zero=True)
        if self._last_time is not None:
            if t < self._last_time - 1e-10:
                raise ValueError("time went backwards; reset controller for another run")
            if abs(t - self._last_time) <= 1e-10:
                return self._last_command.copy()  # idempotent at the same snapshot/time
        p = _vec(_attr(observation, "self_position"), "self_position")
        arrival = _positive(_attr(self._plan, "arrival_time"), "plan.arrival_time", allow_zero=True)
        final_slot = self._base.slot_at(arrival)
        distance = float(np.linalg.norm(final_slot - p))
        og, ng = self._local_gaps(observation)
        ap = self.adaptive_params
        crowded = ((og is not None and og <= ap.obstacle_enter) or
                   (ng is not None and ng <= ap.neighbor_enter))
        clear = ((og is None or og >= ap.obstacle_exit) and
                 (ng is None or ng >= ap.neighbor_exit))
        progress = self._progress(t, distance)
        far_from_goal = distance > ap.terminal_tolerance
        stalled = far_from_goal and progress is not None and progress <= ap.stall_progress
        recovered = progress is not None and progress >= ap.recovery_progress
        self._enter_count = self._enter_count + 1 if crowded else 0
        self._exit_count = self._exit_count + 1 if clear else 0
        self._stall_count = self._stall_count + 1 if stalled else 0
        self._recover_count = self._recover_count + 1 if recovered else 0
        if self._mode_since is None:
            self._mode_since = t
        dwell_ok = t - self._mode_since >= ap.min_dwell - 1e-10
        previous = self.mode
        reason = "hold"
        if dwell_ok:
            if (self.mode is not Mode.RECOVERY and
                    self._stall_count >= ap.stall_samples):
                self._switch(Mode.RECOVERY, t)
                reason = "stagnation"
            elif self.mode is Mode.NORMAL and self._enter_count >= ap.enter_samples:
                self._switch(Mode.PASSAGE, t)
                reason = "crowding"
            elif self.mode is Mode.PASSAGE and self._exit_count >= ap.exit_samples:
                self._switch(Mode.NORMAL, t)
                reason = "clearance"
            elif (self.mode is Mode.RECOVERY and
                  self._recover_count >= ap.exit_samples):
                self._switch(Mode.NORMAL if clear else Mode.PASSAGE, t)
                reason = "progress_recovered"
        weight, circulation = self._effective_coefficients()
        # The original C computes radial obstacle and inter-robot repulsion.
        # ONLY formation and circulation coefficients change. NEVER repulsion.
        local_c = FormationVortexController(
            self._base.layout, self._base.plan, self.robot_id,
            replace(self.base_params, w_formation=weight, k_circ=circulation),
            self._base.slot_provider)
        command = local_c.command_at(observation, t)
        self._last_command = command.copy()
        self._last_time = t
        self.records.append(ModeRecord(
            robot_id=self.robot_id, time=t, mode=self.mode.value,
            previous_mode=previous.value, transitioned=self.mode is not previous,
            reason=reason, min_obstacle_gap=og, min_neighbor_gap=ng,
            distance_to_final_slot=distance, progress_window=progress,
            crowded=crowded, stalled=stalled, w_formation=weight,
            k_circ=circulation, circulation_sign=self.base_params.circulation_sign,
            command_x=float(command[0]), command_y=float(command[1])))
        return command.copy()

    def command(self, observation: object) -> np.ndarray:
        return self.command_at(observation, float(_attr(observation, "time", "t")))
