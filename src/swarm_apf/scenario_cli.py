"""Render E2 geometry, not controller trajectories."""
import argparse
from pathlib import Path
from .scenarios_e2 import SCENARIOS, make_case
from .models import SwarmConfig
from .experiment_runner import json_dump, write_csv


def render(case, cfg, target):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 7))
    for o in case.obstacles:
        ax.add_patch(plt.Circle(o.center,o.radius,color="gray",alpha=.6))
        ax.add_patch(plt.Circle(o.center,o.radius+cfg.robot_radius,fill=False,
                               edgecolor="gray",linestyle=":",linewidth=.6))
    for i,(start,goal) in enumerate(zip(case.initial,case.goals)):
        ax.add_patch(plt.Circle(start,cfg.robot_radius,color="tab:blue",alpha=.7))
        ax.add_patch(plt.Circle(goal,cfg.robot_radius,fill=False,edgecolor="tab:green"))
    ax.scatter([],[],color="tab:blue",label="Start disks")
    ax.scatter([],[],facecolors="none",edgecolors="tab:green",label="Goal disks")
    ax.set(xlim=(-20,13),ylim=(-11,11),aspect="equal",xlabel="x",ylabel="y",
           title=f"{case.scenario} N={case.n} seed={case.seed} | individual routes: {case.scene_dict['audit']['all_routes']}")
    ax.legend()
    ax.grid(alpha=.2)
    fig.savefig(target,dpi=120,bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--scenarios",nargs="+",choices=SCENARIOS,default=list(SCENARIOS[:6]))
    parser.add_argument("--robots",nargs="+",type=int,choices=[5,10,20],default=[5,10,20])
    parser.add_argument("--seeds",nargs="+",type=int,default=[0])
    args = parser.parse_args(argv)
    if args.out.exists():
        raise FileExistsError(args.out)
    for values in (args.scenarios,args.robots,args.seeds):
        if len(set(values)) != len(values):
            parser.error("duplicate factor levels")
    cfg = SwarmConfig(max_speed=.8)
    cases = [make_case(s,n,seed,cfg) for s in args.scenarios for n in args.robots for seed in args.seeds]
    args.out.mkdir(parents=True)
    rows = []
    for case in cases:
        stem = f"{case.scenario}_N{case.n}_seed{case.seed}"
        render(case,cfg,args.out/(stem+".png"))
        json_dump(args.out/(stem+".json"),dict(scene_hash=case.scene_hash,**case.scene_dict))
        rows.append(dict(scenario=case.scenario,n=case.n,seed=case.seed,
                         scene_hash=case.scene_hash,**case.scene_dict["audit"]))
    write_csv(args.out/"geometry_audit.csv",rows,list(rows[0]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
