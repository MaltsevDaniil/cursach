"""Free-space factorial sensitivity with a common simulated time budget."""
import argparse
from collections import defaultdict
import math
from pathlib import Path
from .diagnose_free import spacing_case
from .experiment_runner import run_case, code_fingerprint, json_dump, write_csv, aggregate
from .models import SwarmConfig
from .scenarios_e2 import make_case


def steps_for(duration, dt):
    if not math.isfinite(duration) or duration <= 0 or not math.isfinite(dt) or dt <= 0:
        raise ValueError("positive finite duration/dt required")
    steps = round(duration/dt)
    if steps < 1 or not math.isclose(steps*dt,duration,rel_tol=1e-10,abs_tol=1e-10):
        raise ValueError("duration must be an integer multiple of every dt")
    return steps


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--spacings",type=float,nargs="+",default=[.65,1.4])
    parser.add_argument("--dts",type=float,nargs="+",default=[.025,.05])
    parser.add_argument("--seeds",type=int,nargs="+",default=[0,1])
    parser.add_argument("--duration",type=float,default=80.)
    args = parser.parse_args(argv)
    if args.out.exists():
        raise FileExistsError(args.out)
    for values in (args.spacings,args.dts,args.seeds):
        if len(values) != len(set(values)):
            parser.error("duplicate factor levels")
    if any(seed < 0 for seed in args.seeds):
        parser.error("seeds must be nonnegative")
    cases = []
    for dt in args.dts:
        steps = steps_for(args.duration,dt)
        cfg = SwarmConfig(dt=dt,max_speed=.8)
        for seed in args.seeds:
            base = make_case("free",5,seed,cfg)
            for spacing in args.spacings:
                cases.append((cfg,steps,spacing_case(base,cfg,spacing)))
    args.out.mkdir(parents=True)
    code = code_fingerprint()
    manifest = dict(kind="free_spacing_dt_sensitivity",run_status="IN_PROGRESS",code=code,
                    settings={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
                    n=5, note="reconstructed A/B; fixed duration; not canonical E2")
    json_dump(args.out/"manifest.json",manifest)
    rows = []
    for cfg,steps,case in cases:
        spacing = case.layout.spacing
        for method in "ABCD":
            target = args.out/f"dt_{cfg.dt}/spacing_{spacing}/seed_{case.seed}/{method}"
            row = run_case(method,case,cfg,max_steps=steps,allow_proxies=True,target=target,code=code)
            row.update(dt=cfg.dt,spacing=spacing,duration=args.duration)
            rows.append(row)
            write_csv(args.out/"runs.csv",rows,list(rows[0]))
            print(cfg.dt,spacing,case.seed,method,row["status"],flush=True)
    groups = defaultdict(list)
    for row in rows:
        groups[row["dt"],row["spacing"]].append(row)
    totals = []
    for (dt,spacing),group in sorted(groups.items()):
        totals += [dict(dt=dt,spacing=spacing,**r) for r in aggregate(group)]
    write_csv(args.out/"aggregate.csv",totals,list(totals[0]))
    json_dump(args.out/"aggregate.json",totals)
    if code_fingerprint()["fingerprint"] != code["fingerprint"]:
        raise RuntimeError("source changed during run")
    manifest.update(run_status="COMPLETE",completed_runs=len(rows))
    json_dump(args.out/"manifest.json",manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
