"""Controlled free-space spacing diagnostic; never edits E2 or controller defaults."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
from pathlib import Path
import numpy as np
from .experiment_runner import run_case, code_fingerprint, json_dump, write_csv, sha
from .scenarios_e2 import make_case
from .formation import FormationLayout, slot_position
from .models import SwarmConfig
from .sensing import observe
from .models import WorldState
from .multi_apf import APFController


def goal_forces(goals, cfg):
    """Offline test: is the exact desired configuration a force equilibrium?"""
    world = WorldState(tuple(range(len(goals))), goals, np.zeros_like(goals))
    return [APFController().forces(observe(world,i,goals,(),cfg)).total.tolist()
            for i in range(len(goals))]


def motion_summary(positions, times, window=5.):
    positions, times = np.asarray(positions), np.asarray(times)
    mask = times >= times[-1]-window-1e-9
    points = positions[mask]
    path = np.linalg.norm(np.diff(points,axis=0),axis=2).sum(axis=0)
    displacement = np.linalg.norm(points[-1]-points[0],axis=1)
    return dict(window_s=float(times[-1]-times[mask][0]),
                path_per_robot=path.tolist(), displacement_per_robot=displacement.tolist(),
                displacement_to_path=np.divide(displacement,path,out=np.zeros_like(path),where=path>0).tolist())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--spacings",nargs="+",type=float,default=[.65,1.4])
    parser.add_argument("--max-steps",type=int,default=1600)
    args = parser.parse_args(argv)
    if args.out.exists():
        raise FileExistsError(args.out)
    if args.max_steps < 0 or len(set(args.spacings)) != len(args.spacings):
        parser.error("nonnegative steps and unique spacings required")
    cfg = SwarmConfig(max_speed=.8)
    base = make_case("free",5,0,cfg)
    cases = []
    base_slots = np.array([slot_position(base.layout,base.plan.at(0),i) for i in base.layout.robot_ids])
    jitter = base.initial-base_slots
    for spacing in args.spacings:
        layout = FormationLayout(base.layout.robot_ids,"line",spacing)
        initial = np.array([slot_position(layout,base.plan.at(0),i) for i in layout.robot_ids])+jitter
        goals = np.array([slot_position(layout,base.plan.at(base.plan.arrival_time),i) for i in layout.robot_ids])
        from .swarm import SwarmSimulator
        SwarmSimulator(cfg).reset(initial,goals)
        scene = dict(base.scene_dict,layout=asdict(layout),positions=initial.tolist(),goals=goals.tolist(),
                     diagnostic="spacing sensitivity, not canonical E2",
                     audit=None)
        cases.append(replace(base,layout=layout,initial=initial,goals=goals,scene_dict=scene,scene_hash=sha(scene)))
    args.out.mkdir(parents=True)
    code = code_fingerprint()
    manifest = dict(kind="free_spacing_diagnostic",run_status="IN_PROGRESS",code=code,
                    settings=dict(spacings=args.spacings,max_steps=args.max_steps,n=5,seed=0))
    json_dump(args.out/"manifest.json",manifest)
    rows = []
    for case in cases:
        spacing = case.layout.spacing
        json_dump(args.out/f"spacing_{spacing}/goal_forces.json",
                  dict(forces_at_exact_slots=goal_forces(case.goals,cfg),
                       note="A command is zero at exact goal despite possible nonzero force; B/C remain active"))
        for method in "ABCD":
            target = args.out/f"spacing_{spacing}"/method
            row = run_case(method,case,cfg,max_steps=args.max_steps,allow_proxies=True,target=target,code=code)
            import csv
            with (target/"history.csv").open() as stream:
                history = list(csv.DictReader(stream))
            positions = np.array([[float(r["x"]),float(r["y"])] for r in history]).reshape(-1,5,2)
            times = np.array([float(r["time"]) for r in history[::5]])
            diagnostics = motion_summary(positions,times)
            if method == "D":
                with (target/"mode_log.csv").open() as stream:
                    logs = list(csv.DictReader(stream))
                diagnostics["mode_samples"] = dict(Counter(r["mode"] for r in logs))
                diagnostics["transition_reasons"] = dict(Counter(r["reason"] for r in logs if r["transitioned"]=="True"))
            json_dump(target/"diagnostics.json",diagnostics)
            row["spacing"] = spacing
            rows.append(row)
            print(spacing,method,row["status"],row["arrived_agent_count"],flush=True)
            write_csv(args.out/"runs.csv",rows,list(rows[0]))
    if code_fingerprint()["fingerprint"] != code["fingerprint"]:
        raise RuntimeError("source changed")
    manifest.update(run_status="COMPLETE",completed_runs=len(rows))
    json_dump(args.out/"manifest.json",manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
