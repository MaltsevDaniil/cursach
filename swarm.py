"""E1 synchronous swarm orchestrator. No collision avoidance / rescue here."""
from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Sequence

import numpy as np

from .geometry import Obstacle
from .models import SwarmConfig, WorldState, coordinates
from .physics import CollisionEvent, check_sweeps
from .sensing import observe
from .swarm_controller import GoalController, SwarmController


@dataclass(frozen=True)
class RejectedStep:
    start: np.ndarray
    proposed: np.ndarray
    commands: np.ndarray
    events: tuple[CollisionEvent, ...]
    time: float                          # time of the rejected proposal's start


@dataclass(frozen=True)
class StepResult:
    status: str
    accepted: bool
    attempted_steps: int
    accepted_steps: int
    events: tuple[CollisionEvent, ...]


@dataclass(frozen=True)
class RunResult:
    status: str
    robot_ids: tuple[int, ...]
    goals: np.ndarray
    obstacles: tuple[Obstacle, ...]
    config: SwarmConfig
    times: np.ndarray                   # (accepted_steps + 1,)
    positions: np.ndarray               # (accepted_steps + 1, N, 2)
    velocities: np.ndarray              # (accepted_steps + 1, N, 2)
    commands: np.ndarray                # (accepted_steps, N, 2)
    accepted_steps: int
    attempted_steps: int
    events: tuple[CollisionEvent, ...]
    rejected_step: RejectedStep | None
    min_robot_clearance: float
    min_obstacle_clearance: float

    @property
    def success(self) -> bool:
        return self.status == "SUCCESS"

    @property
    def path_lengths(self) -> np.ndarray:
        return np.linalg.norm(np.diff(self.positions, axis=0), axis=2).sum(axis=0)


def _safe_number(x: float):
    return float(x) if np.isfinite(x) else None


class SwarmSimulator:
    """Finite-N, pure-Euler, shared snapshot, atomic step commit on no collision.

    reset raises ValueError for invalid start, goal, or parameter configuration.
    step/run use statuses RUNNING, SUCCESS, TIMEOUT, COLLISION_* or NUMERIC_ERROR.
    """

    def __init__(self, config: SwarmConfig | None = None,
                 controller: SwarmController | None = None):
        self.config = config or SwarmConfig()
        self.controller = controller or GoalController()
        self._world: WorldState | None = None
        self._goals: np.ndarray | None = None
        self._obstacles: tuple[Obstacle, ...] = ()
        self._times: list[float] = []
        self._positions: list[np.ndarray] = []
        self._velocities: list[np.ndarray] = []
        self._commands: list[np.ndarray] = []
        self._events: tuple[CollisionEvent, ...] = ()
        self._rejected: RejectedStep | None = None
        self._status = "NOT_INITIALIZED"
        self._attempted = 0
        self._min_robot = float("inf")
        self._min_obstacle = float("inf")

    @property
    def state(self) -> WorldState:
        if self._world is None:
            raise RuntimeError("call reset first")
        return self._world

    @property
    def status(self) -> str:
        return self._status

    def reset(self, positions, goals, obstacles: Sequence[Obstacle] = (), *,
              robot_ids: Sequence[int] | None = None, initial_velocities=None) -> WorldState:
        """Validate and canonically sort IDs; no partial state on failed reset."""
        p = coordinates(positions)
        g = coordinates(goals, label="goals")
        if p.shape != g.shape:
            raise ValueError("each robot must have one individual goal")
        n = len(p)
        ids = tuple(range(n)) if robot_ids is None else tuple(robot_ids)
        if len(ids) != n or len(set(ids)) != n or any(
                isinstance(i, bool) or not isinstance(i, (int, np.integer)) for i in ids):
            raise ValueError("robot_ids must be N unique integers")
        vel = (np.zeros_like(p) if initial_velocities is None
               else coordinates(initial_velocities, label="initial_velocities"))
        if vel.shape != p.shape:
            raise ValueError("initial_velocities must have shape (N, 2)")
        if np.any(np.linalg.norm(vel, axis=1) > self.config.max_speed + 1e-10):
            raise ValueError("initial velocity exceeds max_speed")
        ordered = sorted(range(n), key=lambda i: ids[i])
        ids = tuple(int(ids[i]) for i in ordered)
        p, g, vel = p[ordered], g[ordered], vel[ordered]
        obs = tuple(obstacles)
        if any(not isinstance(o, Obstacle) for o in obs):
            raise ValueError("obstacles must be Obstacle instances")
        cfg = self.config
        start_check = check_sweeps(p, p, ids, obs, cfg.robot_radius,
                                   0.0, cfg.dt, cfg.epsilon_geom)
        goal_check = check_sweeps(g, g, ids, obs, cfg.robot_radius,
                                  0.0, cfg.dt, cfg.epsilon_geom)
        if start_check.events:
            raise ValueError(f"INVALID_SCENE: initial contact {start_check.events[0]}")
        if goal_check.events:
            raise ValueError(f"INVALID_SCENE: overlapping/unreachable goals {goal_check.events[0]}")
        world = WorldState(ids, p, vel, 0.0)
        # Commit reset only after ALL checks passed.
        self._world = world
        self._goals = g.copy()
        self._goals.setflags(write=False)
        self._obstacles = obs
        self._times = [0.0]
        self._positions = [p.copy()]
        self._velocities = [vel.copy()]
        self._commands = []
        self._events = ()
        self._rejected = None
        self._attempted = 0
        self._min_robot = start_check.min_robot_clearance
        self._min_obstacle = start_check.min_obstacle_clearance
        self._status = "SUCCESS" if self._at_goals(world.positions) else "RUNNING"
        return world

    def _at_goals(self, positions: np.ndarray) -> bool:
        assert self._goals is not None
        return bool(np.all(np.linalg.norm(positions - self._goals, axis=1)
                           <= self.config.goal_tolerance))

    def step(self) -> StepResult:
        if self._world is None:
            raise RuntimeError("call reset first")
        if self._status != "RUNNING":
            raise RuntimeError(f"cannot step terminal state {self._status}; reset first")
        old = self._world
        self._attempted += 1
        try:
            # Snapshot and *all* observations are computed before any command.
            observations = tuple(observe(old, i, self._goals, self._obstacles, self.config)
                                 for i in range(len(old.robot_ids)))
            commands = np.asarray([self.controller.command(o) for o in observations], dtype=float)
            if commands.shape != old.positions.shape or not np.all(np.isfinite(commands)):
                raise ValueError("controller returned nonfinite/wrong-shaped commands")
            # Strict contract: no hidden speed clipping or safety filter.
            if np.any(np.linalg.norm(commands, axis=1) > self.config.max_speed + 1e-10):
                raise ValueError("controller exceeded max_speed")
            proposed = old.positions + self.config.dt * commands
            if not np.all(np.isfinite(proposed)):
                raise ValueError("Euler candidate is nonfinite")
        except (ValueError, TypeError, FloatingPointError, OverflowError):
            self._status = "NUMERIC_ERROR"
            return StepResult(self._status, False, self._attempted,
                              len(self._commands), self._events)

        check = check_sweeps(old.positions, proposed, old.robot_ids,
                             self._obstacles, self.config.robot_radius,
                             old.time, self.config.dt, self.config.epsilon_geom)
        self._min_robot = min(self._min_robot, check.min_robot_clearance)
        self._min_obstacle = min(self._min_obstacle, check.min_obstacle_clearance)
        if check.events:
            # Never commit any of the robots; keep rejected candidate for audit.
            self._events = check.events
            self._rejected = RejectedStep(old.positions.copy(), proposed.copy(),
                                          commands.copy(), check.events, old.time)
            self._status = check.events[0].kind
            return StepResult(self._status, False, self._attempted,
                              len(self._commands), self._events)

        # Atomic Euler commit of all N robots after global collision check.
        self._world = WorldState(old.robot_ids, proposed, commands, old.time + self.config.dt)
        self._commands.append(commands.copy())
        self._times.append(self._world.time)
        self._positions.append(proposed.copy())
        self._velocities.append(commands.copy())
        if self._at_goals(proposed):
            self._status = "SUCCESS"
        return StepResult(self._status, True, self._attempted,
                          len(self._commands), ())

    def run(self, max_steps: int = 2400) -> RunResult:
        if self._world is None:
            raise RuntimeError("call reset first")
        if isinstance(max_steps, bool) or not isinstance(max_steps, int) or max_steps < 0:
            raise ValueError("max_steps must be an integer >= 0")
        for _ in range(max_steps):
            if self._status != "RUNNING":
                break
            self.step()
        if self._status == "RUNNING":
            self._status = "TIMEOUT"
        return self.result()

    def result(self) -> RunResult:
        if self._world is None or self._goals is None:
            raise RuntimeError("call reset first")
        return RunResult(
            self._status, self._world.robot_ids, self._goals.copy(), self._obstacles,
            self.config, np.asarray(self._times).copy(), np.stack(self._positions),
            np.stack(self._velocities),
            np.stack(self._commands) if self._commands else
            np.empty((0, len(self._world.robot_ids), 2)),
            len(self._commands), self._attempted, self._events, self._rejected,
            self._min_robot, self._min_obstacle)

    def save_history(self, directory: str | Path) -> tuple[Path, Path]:
        """Write history.csv (accepted states only) and summary.json (incl. rejected)."""
        result = self.result()
        target = Path(directory)
        target.mkdir(parents=True, exist_ok=True)
        csv_path = target / "history.csv"
        json_path = target / "summary.json"
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["step", "time", "robot_id", "x", "y", "vx", "vy", "goal_x", "goal_y"])
            for t, instant in enumerate(result.times):
                for i, robot_id in enumerate(result.robot_ids):
                    x, y = result.positions[t, i]
                    vx, vy = result.velocities[t, i]
                    gx, gy = result.goals[i]
                    writer.writerow([t, instant, robot_id, x, y, vx, vy, gx, gy])

        summary = {
            "stage": "E1", "status": result.status, "success": result.success,
            "config": asdict(result.config), "robot_ids": list(result.robot_ids),
            "goals": result.goals.tolist(),
            "obstacles": [{"center": o.center, "radius": o.radius} for o in result.obstacles],
            "accepted_steps": result.accepted_steps,
            "attempted_steps": result.attempted_steps,
            "last_accepted_time": float(result.times[-1]),
            "path_lengths": result.path_lengths.tolist(),
            "min_robot_clearance": _safe_number(result.min_robot_clearance),
            "min_obstacle_clearance": _safe_number(result.min_obstacle_clearance),
            "events": [asdict(e) for e in result.events],
            "rejected_step": None if result.rejected_step is None else {
                "time": result.rejected_step.time,
                "start": result.rejected_step.start.tolist(),
                "proposed": result.rejected_step.proposed.tolist(),
                "commands": result.rejected_step.commands.tolist(),
            },
        }
        json_path.write_text(json.dumps(summary, ensure_ascii=False,
                                        indent=2, allow_nan=False) + "\n", encoding="utf-8")
        return csv_path, json_path
