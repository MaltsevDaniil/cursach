"""Reproducible E1 factorial benchmark, with strict A/B provenance checks.

Run as: PYTHONPATH=src python -m swarm_apf.experiment_runner ...
No global state is passed to any agent. E1 handles all motion/collision physics.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import csv
from dataclasses import asdict, dataclass, is_dataclass
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np

from .adaptive_d import AdaptiveDController, AdaptiveDParameters, D_VERSION
from .contract_proxies import AContractProxy, AProxyParameters, BContractProxy
from .geometry import Obstacle
from .models import SwarmConfig
from .swarm import SwarmSimulator
from .vortex_c import FormationVortexController, VortexCParameters, C_VERSION

BENCHMARK_VERSION = 'EXPERIMENT-E1-1.0.0'
SCENARIOS = ('free', 'single_obstacle', 'symmetric_obstacles', 'open_corridor', 'offset_cluster')
SCENE_IDS = {name: i + 1 for i, name in enumerate(SCENARIOS)}
METHODS = ('A', 'B', 'C', 'D')
SCHEMA_VERSION = 'experiment-v1'


def canonical_json(data) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)


def sha(data) -> str:
    return hashlib.sha256(canonical_json(data).encode('utf8')).hexdigest()


def json_dump(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True,
                               allow_nan=False) + '\n', encoding='utf8')


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf8') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def nullable(x: float) -> float | None:
    return float(x) if math.isfinite(float(x)) else None


def take_import(module: str):
    try:
        return importlib.import_module('.' + module, __package__)
    except ModuleNotFoundError as err:
        # Only a wholly missing requested module is a reason to use a proxy.
        if err.name == f'{__package__}.{module}':
            return None
        raise


def formation_api():
    """Use genuine formation.py when supplied; else visibly mark reconstructed geometry."""
    mod = take_import('formation')
    if mod is not None:
        return mod, 'original_formation_module'
    from . import formation_contract
    return formation_contract, 'documented_formation_geometry_reconstruction'


@dataclass(frozen=True)
class Case:
    scenario: str
    n: int
    seed: int
    layout: object
    plan: object
    slot_provider: object
    formation_source: str
    initial: np.ndarray
    goals: np.ndarray
    obstacles: tuple[Obstacle, ...]
    scene_dict: dict
    scene_hash: str


def obstacles_for(name: str) -> tuple[Obstacle, ...]:
    if name == 'free':
        circles = []
    elif name == 'single_obstacle':
        circles = [(0., 0., .65)]
    elif name == 'symmetric_obstacles':
        circles = [(0., -1.0, .55), (0., 1.0, .55)]
    elif name == 'open_corridor':
        # These are short, open rows, NOT impenetrable infinitely long walls.
        circles = [(x, y, .40) for x in (-1., 0., 1.) for y in (-2.1, 2.1)]
    elif name == 'offset_cluster':
        circles = [(-.5, -2.1, .55), (.2, 2.3, .55), (1.0, -.2, .48)]
    else:
        raise ValueError(f'unknown scenario {name}')
    return tuple(Obstacle((x, y), r) for x, y, r in circles)


def make_case(scenario: str, n: int, seed: int, cfg: SwarmConfig,
              *, jitter: float = .015, shape: str = 'auto', spacing: float = 1.4,
              anchor_speed: float = .5) -> Case:
    if scenario not in SCENARIOS or n < 2 or seed < 0 or jitter < 0:
        raise ValueError('invalid scenario/n/seed/jitter')
    formation, source = formation_api()
    selected_shape = ('line' if n <= 5 else 'V') if shape == 'auto' else shape
    ids = tuple(range(n))
    layout = formation.FormationLayout(ids, kind=selected_shape, spacing=spacing)
    plan = formation.FormationPlan(start_anchor=(-4., 0.), group_goal=(4., 0.),
                                   cruise_speed=anchor_speed)
    provider = formation.slot_position
    start = np.stack([provider(layout, plan.at(0.), rid) for rid in ids]).astype(float)
    goals = np.stack([provider(layout, plan.at(plan.arrival_time), rid)
                      for rid in ids]).astype(float)
    # The same case seed yields identical initial conditions for every controller.
    rng = np.random.default_rng(np.random.SeedSequence([seed, n, SCENE_IDS[scenario], 189173]))
    initial = start + rng.uniform(-jitter, jitter, size=start.shape)
    obstacles = obstacles_for(scenario)
    desc = dict(scenario=scenario, n=n, seed=seed, robot_ids=list(ids),
                positions=initial.tolist(), goals=goals.tolist(),
                obstacles=[dict(center=list(o.center), radius=o.radius) for o in obstacles],
                formation=dict(kind=selected_shape, spacing=spacing,
                               start_anchor=[-4., 0.], group_goal=[4., 0.],
                               anchor_speed=anchor_speed,
                               arrival_time=float(plan.arrival_time), source=source),
                jitter=jitter, physics=asdict(cfg))
    # Preflight scene before any results: invalid geometry must never be quietly skipped.
    check = SwarmSimulator(config=cfg)
    check.reset(initial, goals, obstacles, robot_ids=ids)
    return Case(scenario, n, seed, layout, plan, provider, source,
                initial, goals, obstacles, desc, sha(desc))


def make_agents(method: str, case: Case, allow_proxies: bool):
    ids = tuple(case.layout.robot_ids)
    if method == 'A':
        mod = take_import('multi_apf')
        if mod is None:
            if not allow_proxies:
                raise RuntimeError('A requires frozen multi_apf.py; add --allow-contract-proxies for a marked proxy')
            agents = {i: AContractProxy() for i in ids}
            return agents, 'contract_reconstruction_NOT_original_A', asdict(AProxyParameters())
        agents = {i: mod.APFController(mod.APFParameters()) for i in ids}
        return agents, 'native_multi_apf', asdict(mod.APFParameters())
    if method == 'B':
        mod = take_import('formation')
        if mod is None:
            if not allow_proxies:
                raise RuntimeError('B requires frozen formation.py; add --allow-contract-proxies for a marked proxy')
            agents = {i: BContractProxy(case.layout, case.plan, i, case.slot_provider)
                      for i in ids}
            return agents, 'contract_reconstruction_NOT_original_B', asdict(VortexCParameters(k_circ=0.))
        agents = {i: mod.FormationController(case.layout, case.plan, i) for i in ids}
        sample = agents[ids[0]]
        if hasattr(sample, 'metadata'):
            original_metadata = sample.metadata()
        elif hasattr(sample, 'params') and is_dataclass(sample.params):
            original_metadata = {'parameters': asdict(sample.params)}
        else:
            raise RuntimeError('native B must expose metadata() or dataclass params for reproducible comparison')
        return agents, 'native_formation', original_metadata
    if method == 'C':
        params = VortexCParameters()
        return ({i: FormationVortexController(case.layout, case.plan, i, params,
                                               slot_provider=case.slot_provider) for i in ids},
                'supplied_C_source_unmodified', asdict(params))
    if method == 'D':
        params = VortexCParameters()
        adaptive = AdaptiveDParameters()
        agents = {i: AdaptiveDController(case.layout, case.plan, i, params, adaptive,
                                         slot_provider=case.slot_provider) for i in ids}
        return agents, 'supplied_D_source_unmodified', dict(base=asdict(params), adaptive=asdict(adaptive))
    raise ValueError(method)


class TimedLocalDispatcher:
    """E1's single .command controller dispatches to individual local agents by public ID.

    The only addition to observation is the already-known, common physical radius.
    The dispatcher neither looks at WorldState nor modifies agent commands.
    """
    def __init__(self, agents: dict, radius: float):
        self.agents = agents
        self.radius = radius
        self.records = []

    def command(self, obs):
        view = SimpleNamespace(**vars(obs), robot_radius=self.radius)
        ns0 = time.perf_counter_ns()
        try:
            return self.agents[obs.robot_id].command(view)
        finally:
            self.records.append(dict(time=float(obs.time), robot_id=int(obs.robot_id),
                                     command_ns=time.perf_counter_ns()-ns0))


def formation_trace(result, plan, layout, provider) -> list[dict]:
    records = []
    for k, t in enumerate(result.times):
        slots = np.stack([provider(layout, plan.at(float(t)), rid)
                          for rid in result.robot_ids])
        offsets = result.positions[k] - slots
        tracking = float(np.sqrt(np.mean(np.sum(offsets**2, axis=1))))
        shape = float(np.sqrt(np.mean(np.sum((offsets-offsets.mean(axis=0))**2, axis=1))))
        records.append(dict(step=k, time_s=float(t), tracking_rms=tracking, shape_rms=shape))
    return records


def compute_metrics(result, trace: list[dict], cfg: SwarmConfig,
                    step_rows: list[dict]) -> tuple[dict, list[dict]]:
    """Offline only: never callable by a controller."""
    path = np.asarray(result.path_lengths, dtype=float)
    distances = np.linalg.norm(result.positions[-1] - result.goals, axis=1)
    fractions = np.flatnonzero(distances <= cfg.goal_tolerance)
    centroid = np.mean(result.positions, axis=1)
    centroid_path = float(np.linalg.norm(np.diff(centroid, axis=0), axis=1).sum())
    success = result.status == 'SUCCESS'
    censored = result.status == 'TIMEOUT'  # Collision = competing terminal failure.
    if result.events:
        end_time = min(float(e.time) for e in result.events)
    else:
        end_time = float(result.times[-1])
    min_gap = min(result.min_robot_clearance, result.min_obstacle_clearance)
    controller_ms = [row['controller_step_ms'] for row in step_rows]
    e1_ms = [row['e1_step_ms'] for row in step_rows]
    metrics = dict(
        status=result.status, success=success, right_censored=censored,
        censoring_reason='administrative_time_limit' if censored else None,
        completion_time_s=float(result.times[-1]) if success else None,
        completion_steps=int(result.accepted_steps) if success else None,
        last_accepted_time_s=float(result.times[-1]),
        observation_end_time_s=end_time,
        attempted_steps=int(result.attempted_steps),
        accepted_steps=int(result.accepted_steps),
        path_length_group_sum=float(path.sum()),
        path_length_group_centroid=centroid_path,
        path_length_robot_mean=float(path.mean()),
        collision_event_count=len(result.events),
        collision_robot_event_count=sum(e.kind == 'COLLISION_ROBOT' for e in result.events),
        collision_obstacle_event_count=sum(e.kind == 'COLLISION_OBSTACLE' for e in result.events),
        min_gap_swept_including_rejected=nullable(min_gap),
        min_gap_robot_swept=nullable(result.min_robot_clearance),
        min_gap_obstacle_swept=nullable(result.min_obstacle_clearance),
        formation_tracking_rms_mean=float(np.mean([r['tracking_rms'] for r in trace])),
        formation_tracking_rms_final=trace[-1]['tracking_rms'],
        formation_shape_rms_mean=float(np.mean([r['shape_rms'] for r in trace])),
        formation_shape_rms_final=trace[-1]['shape_rms'],
        arrived_agent_fraction=float(len(fractions) / len(result.robot_ids)),
        arrived_agent_count=int(len(fractions)),
        mean_controller_step_ms=float(np.mean(controller_ms)) if controller_ms else None,
        p95_controller_step_ms=float(np.percentile(controller_ms, 95)) if controller_ms else None,
        mean_e1_step_ms=float(np.mean(e1_ms)) if e1_ms else None,
        total_e1_step_ms=float(sum(e1_ms)),
    )
    robots = [dict(robot_id=int(rid), path_length=float(path[i]),
                   final_goal_distance=float(distances[i]),
                   reached_goal=bool(distances[i] <= cfg.goal_tolerance))
              for i, rid in enumerate(result.robot_ids)]
    return metrics, robots


def git_head(folder: Path) -> str | None:
    try:
        return subprocess.check_output(['git', '-C', str(folder), 'rev-parse', 'HEAD'],
                                       stderr=subprocess.DEVNULL, timeout=2).decode().strip()
    except (FileNotFoundError, OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None


def code_fingerprint() -> dict:
    folder = Path(__file__).resolve().parent
    paths = sorted(folder.glob('*.py'))
    if not paths:
        raise RuntimeError('no source files present')
    checksums = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    return dict(files=checksums, fingerprint=sha(checksums), git_commit=git_head(folder))


def run_case(method: str, case: Case, cfg: SwarmConfig, *, max_steps: int,
             allow_proxies: bool, target: Path, code: dict) -> dict:
    agents, source, parameters = make_agents(method, case, allow_proxies)
    for agent in agents.values():
        if hasattr(agent, 'reset'):
            agent.reset()
    controller = TimedLocalDispatcher(agents, cfg.robot_radius)
    sim = SwarmSimulator(cfg, controller)
    sim.reset(case.initial.copy(), case.goals.copy(), case.obstacles,
              robot_ids=case.layout.robot_ids)
    steps = []
    for _ in range(max_steps):
        if sim.status != 'RUNNING':
            break
        before = len(controller.records)
        t0 = time.perf_counter_ns()
        step_result = sim.step()  # Real supplied E1 step + swept contact checks.
        elapsed = time.perf_counter_ns() - t0
        calls = controller.records[before:]
        steps.append(dict(attempt=int(step_result.attempted_steps),
                          start_time_s=float(sim.state.time - cfg.dt if step_result.accepted else sim.state.time),
                          accepted=bool(step_result.accepted), status_after=step_result.status,
                          controller_step_ms=sum(r['command_ns'] for r in calls)/1e6,
                          e1_step_ms=elapsed/1e6, command_calls=len(calls)))
    result = sim.run(max_steps=0)  # E1 marks still-running as TIMEOUT, does not step again.
    trace = formation_trace(result, case.plan, case.layout, case.slot_provider)
    met, robots = compute_metrics(result, trace, cfg, steps)
    prefix = dict(method=method, scenario=case.scenario, n=case.n, seed=case.seed,
                  scene_hash=case.scene_hash, controller_source=source,
                  formation_source=case.formation_source,
                  code_fingerprint=code['fingerprint'])
    target.mkdir(parents=True, exist_ok=False)
    sim.save_history(target)  # E1 original history accepted-only, original summary saved below.
    e1_summary = json.loads((target / 'summary.json').read_text(encoding='utf8'))
    write_csv(target / 'step_times.csv', steps,
              ['attempt','start_time_s','accepted','status_after','controller_step_ms',
               'e1_step_ms','command_calls'])
    write_csv(target / 'formation_trace.csv', trace,
              ['step','time_s','tracking_rms','shape_rms'])
    write_csv(target / 'robots.csv', robots,
              ['robot_id','path_length','final_goal_distance','reached_goal'])
    write_csv(target / 'collision_events.csv', [asdict(e) for e in result.events],
              ['kind','robot_ids','obstacle_index','clearance','fraction','time'])
    if result.rejected_step is not None:
        rej = result.rejected_step
        write_csv(target / 'rejected_step.csv', [dict(
            robot_id=int(rid), start_x=float(rej.start[i, 0]),
            start_y=float(rej.start[i, 1]), proposed_x=float(rej.proposed[i, 0]),
            proposed_y=float(rej.proposed[i, 1]), command_x=float(rej.commands[i, 0]),
            command_y=float(rej.commands[i, 1])) for i, rid in enumerate(result.robot_ids)],
            ['robot_id','start_x','start_y','proposed_x','proposed_y','command_x','command_y'])
    if method == 'D':
        log = []
        accepted_times = {round(float(t), 9) for t in result.times[:-1]}
        for agent in agents.values():
            for record in agent.records:
                row = record.as_dict()
                row['command_step_accepted'] = round(float(record.time), 9) in accepted_times
                log.append(row)
        log.sort(key=lambda r: (r['time'], r['robot_id']))
        write_csv(target / 'mode_log.csv', log,
                  list(log[0]) if log else list(agent.records[0].as_dict()) + ['command_step_accepted'] if any(a.records for a in agents.values()) else
                  ['robot_id','time','mode','previous_mode','transitioned','reason','command_step_accepted'])
    summary = dict(schema=SCHEMA_VERSION, benchmark_version=BENCHMARK_VERSION,
                   provenance=prefix, controller_parameters=parameters,
                   physics_parameters=asdict(cfg), scene=case.scene_dict,
                   metrics=met, per_robot=robots, e1_original_summary=e1_summary)
    json_dump(target / 'summary.json', summary)
    return dict(**prefix, **met)


def aggregate(rows: list[dict]) -> list[dict]:
    groups = defaultdict(list)
    for row in rows:
        groups[row['method'], row['scenario'], row['n']].append(row)
    out = []
    for (method, scenario, n), group in sorted(groups.items()):
        usable = [r for r in group if r['status'] in
                  ('SUCCESS', 'TIMEOUT', 'COLLISION_ROBOT', 'COLLISION_OBSTACLE')]
        successes = [r for r in usable if r['success']]
        out.append(dict(method=method, scenario=scenario, n=n,
                        runs_total=len(group), runs_valid=len(usable),
                        runs_invalid=len(group)-len(usable),
                        successes=len(successes), success_rate=(len(successes)/len(usable)
                                                                if usable else None),
                        timeouts=sum(r['status']=='TIMEOUT' for r in group),
                        collision_runs=sum(r['status'].startswith('COLLISION_') for r in group),
                        mean_completion_time_success_only_s=(statistics.mean(
                            r['completion_time_s'] for r in successes) if successes else None),
                        mean_group_path_all_terminal_outcomes=(statistics.mean(
                            r['path_length_group_sum'] for r in usable) if usable else None)))
    return out


def get_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--algorithms', nargs='+', choices=METHODS, default=list(METHODS))
    parser.add_argument('--scenarios', nargs='+', choices=SCENARIOS, default=list(SCENARIOS))
    parser.add_argument('--robots', nargs='+', type=int, default=[5, 10, 20])
    parser.add_argument('--seeds', nargs='+', type=int, default=list(range(10)))
    parser.add_argument('--max-steps', type=int, default=540)
    parser.add_argument('--dt', type=float, default=.05)
    parser.add_argument('--max-speed', type=float, default=.8)
    parser.add_argument('--robot-radius', type=float, default=.2)
    parser.add_argument('--neighbor-radius', type=float, default=2.5)
    parser.add_argument('--obstacle-sensor-radius', type=float, default=3.)
    parser.add_argument('--goal-tolerance', type=float, default=.25)
    parser.add_argument('--epsilon-geom', type=float, default=1e-9)
    parser.add_argument('--jitter', type=float, default=.015)
    parser.add_argument('--formation', choices=['auto','line','V'], default='auto')
    parser.add_argument('--spacing', type=float, default=1.4)
    parser.add_argument('--anchor-speed', type=float, default=.5)
    parser.add_argument('--allow-contract-proxies', action='store_true',
                        help='EXPLICIT opt-in: missing genuine A/B become marked formula reconstructions')
    parser.add_argument('--expect-code-sha', type=str, default=None,
                        help='abort when source-code fingerprint is different from expected')
    parser.add_argument('--out', type=Path, required=True,
                        help='new, nonexisting output directory; never silently overwritten')
    return parser


def main(argv=None) -> int:
    args = get_parser().parse_args(argv)
    if args.max_steps < 0 or any(n < 2 for n in args.robots) or any(s < 0 for s in args.seeds):
        raise ValueError('max_steps >=0, robots>=2, seeds>=0 required')
    if len(set(args.algorithms)) != len(args.algorithms) or len(set(args.scenarios)) != len(args.scenarios) or len(set(args.robots)) != len(args.robots) or len(set(args.seeds)) != len(args.seeds):
        raise ValueError('duplicate factor levels are not allowed')
    cfg = SwarmConfig(dt=args.dt, max_speed=args.max_speed, robot_radius=args.robot_radius,
                      neighbor_radius=args.neighbor_radius,
                      obstacle_sensor_radius=args.obstacle_sensor_radius,
                      goal_tolerance=args.goal_tolerance, epsilon_geom=args.epsilon_geom)
    code = code_fingerprint()
    if args.expect_code_sha is not None and code['fingerprint'] != args.expect_code_sha:
        raise RuntimeError('code fingerprint mismatch; refusing an uncontrolled benchmark')
    if args.out.exists():
        raise FileExistsError(f'{args.out} already exists; choose a fresh output directory')
    # Fail closed for unavailable true A and B (unless explicitly permitted).
    for alg, mod in (('A','multi_apf'), ('B','formation')):
        if alg in args.algorithms and not args.allow_contract_proxies and take_import(mod) is None:
            raise RuntimeError(f'missing original {mod}.py for {alg}; use --allow-contract-proxies ONLY for provisional experiments')
    cases = [make_case(scenario, n, seed, cfg, jitter=args.jitter, shape=args.formation,
                       spacing=args.spacing, anchor_speed=args.anchor_speed)
             for scenario in args.scenarios for n in args.robots for seed in args.seeds]
    params = dict(algorithms=args.algorithms, scenarios=args.scenarios, robots=args.robots,
                  seeds=args.seeds, max_steps=args.max_steps, physics=asdict(cfg),
                  jitter=args.jitter, formation=args.formation, spacing=args.spacing,
                  anchor_speed=args.anchor_speed, allow_contract_proxies=args.allow_contract_proxies)
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(schema=SCHEMA_VERSION, benchmark_version=BENCHMARK_VERSION,
                    run_status='IN_PROGRESS', code=code, settings=params,
                    python=sys.version, numpy=np.__version__, platform=platform.platform(),
                    note='Code+scenes deterministic; wall clock timings are NOT bit-reproducible')
    json_dump(output / 'manifest.json', manifest)
    rows = []
    for case in cases:
        scene_rel = Path('scenes') / case.scenario / f'N{case.n}' / f'seed_{case.seed:04d}.json'
        json_dump(output / scene_rel, dict(hash=case.scene_hash, **case.scene_dict))
        for alg in args.algorithms:
            target = output / 'runs' / alg / case.scenario / f'N{case.n}' / f'seed_{case.seed:04d}'
            row = run_case(alg, case, cfg, max_steps=args.max_steps,
                           allow_proxies=args.allow_contract_proxies, target=target, code=code)
            rows.append(row)
            print(f"{alg} {case.scenario} N={case.n} seed={case.seed}: {row['status']}", flush=True)
            # Running scoreboard is valid even if an unexpected error interrupts the suite.
            write_csv(output / 'runs.csv', rows, list(rows[0]))
    summary = aggregate(rows)
    write_csv(output / 'aggregate.csv', summary, list(summary[0]) if summary else
              ['method','scenario','n','runs_total','success_rate'])
    json_dump(output / 'aggregate.json', summary)
    if code_fingerprint()['fingerprint'] != code['fingerprint']:
        raise RuntimeError('source changed during run: results must not be treated as frozen')
    manifest['run_status'] = 'COMPLETE'
    manifest['completed_runs'] = len(rows)
    json_dump(output / 'manifest.json', manifest)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
