"""E2 preflight and benchmark, kept separate from historical E1."""
import argparse
from pathlib import Path
from . import experiment_runner as runner
from .scenarios_e2 import SCENARIOS, VERSION, make_case, cohort
from .models import SwarmConfig


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS[:6]))
    parser.add_argument("--robots", nargs="+", type=int, choices=[5,10,20], default=[5,10,20])
    parser.add_argument("--seeds", nargs="+", type=int, default=[0])
    parser.add_argument("--algorithms", nargs="+", choices=runner.METHODS, default=list(runner.METHODS))
    parser.add_argument("--max-steps", type=int, default=1600)
    parser.add_argument("--robot-radius", type=float, default=.2)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--stress-only", action="store_true")
    parser.add_argument("--allow-contract-proxies", action="store_true")
    args = parser.parse_args(argv)
    if args.out.exists():
        raise FileExistsError(args.out)
    if args.max_steps < 0 or any(s < 0 for s in args.seeds):
        parser.error("nonnegative steps and seeds required")
    for levels in (args.scenarios,args.robots,args.seeds,args.algorithms):
        if len(levels) != len(set(levels)):
            parser.error("duplicate factor levels")
    stress = args.scenarios == ["blocked_room"]
    if args.stress_only != stress or ("blocked_room" in args.scenarios and not stress):
        parser.error("blocked_room requires --stress-only and cannot be mixed")
    if not args.preflight_only and not args.allow_contract_proxies and any(a in ("A","B") for a in args.algorithms):
        parser.error("reconstructed A/B require --allow-contract-proxies")
    cfg = SwarmConfig(max_speed=.8, robot_radius=args.robot_radius)
    cases = [make_case(s,n,seed,cfg) for s in args.scenarios for n in args.robots for seed in args.seeds]
    code = runner.code_fingerprint()
    args.out.mkdir(parents=True)
    settings = {k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}
    manifest = dict(version=VERSION, code=code, settings=settings, run_status="IN_PROGRESS")
    runner.json_dump(args.out/"manifest.json", manifest)
    for case in cases:
        runner.json_dump(args.out/f"scenes/{case.scenario}/N{case.n}/seed_{case.seed:04d}.json",
                         dict(scene_hash=case.scene_hash, **case.scene_dict))
    runner.json_dump(args.out/"preflight.json", [dict(scenario=c.scenario,n=c.n,seed=c.seed,**c.scene_dict["audit"]) for c in cases])
    rows = []
    if not args.preflight_only:
        for case in cases:
            for method in args.algorithms:
                row = runner.run_case(method,case,cfg,max_steps=args.max_steps,
                    allow_proxies=args.allow_contract_proxies,
                    target=args.out/f"runs/{method}/{case.scenario}/N{case.n}/seed_{case.seed:04d}",code=code)
                row["cohort"] = cohort(case.scenario)
                rows.append(row)
                print(method, case.scenario, case.n, case.seed, row["status"], flush=True)
                selected = [r for r in rows if r["cohort"] == row["cohort"]]
                prefix = {"benchmark":"","diagnostic":"diagnostic_","stress":"stress_"}[row["cohort"]]
                runner.write_csv(args.out/f"{prefix}runs.csv", selected, list(selected[0]))
                totals = runner.aggregate(selected)
                runner.write_csv(args.out/f"{prefix}aggregate.csv", totals, list(totals[0]))
                runner.json_dump(args.out/f"{prefix}aggregate.json", totals)
    if runner.code_fingerprint()["fingerprint"] != code["fingerprint"]:
        raise RuntimeError("source changed during run")
    manifest.update(run_status="PREFLIGHT_COMPLETE" if args.preflight_only else "COMPLETE",
                    completed_cases=len(cases), completed_runs=len(rows))
    runner.json_dump(args.out/"manifest.json", manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
