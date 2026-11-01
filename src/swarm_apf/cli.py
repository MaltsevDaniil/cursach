"""CLI for regression experiments. No global RNG mutations."""

import argparse
import csv
import json
from pathlib import Path
from .planner import plan_path
from .scenarios import scenarios
from .visualization import save_path_figure


LEGACY_PARAMS = dict(step=0.08, max_iterations=5000, k_att=1.5,
                     k_rep=2.5, d0=2.5, k_circ=0.8, goal_tolerance=0.15)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Single-agent APF+vortex (E0)")
    parser.add_argument("--scenario", choices=["all", *map(str, range(1, 8))],
                        default="all")
    parser.add_argument("--out", type=Path, default=Path("results"))
    parser.add_argument("--seed", type=int, default=42,
                        help="Recorded for future experiments; E0 is deterministic")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    chosen = scenarios()
    if args.scenario != "all":
        chosen = {int(args.scenario): chosen[int(args.scenario)]}
    rows = []
    configuration = {
        "version": "0.1.0",
        "seed": args.seed,
        "uses_randomness": False,
        "params": LEGACY_PARAMS,
        "agent_radius": 0.0,
        "force_softening": 0.05,
        "max_repulsion": 10000.0,
        "epsilon_geom": 1e-9,
        "stagnation_window": 40,
        "stagnation_displacement": 0.18,
        "stagnation_ratio": 0.2,
        "scenarios": {
            str(n): {
                "name": scene.name,
                "start": scene.start,
                "goal": scene.goal,
                "obstacles": [{"center": o.center, "radius": o.radius}
                              for o in scene.obstacles],
            } for n, scene in chosen.items()
        },
    }
    with (args.out / "config.json").open("w", encoding="utf-8") as f:
        json.dump(configuration, f, indent=2, allow_nan=False)
    for number, scene in chosen.items():
        result = plan_path(scene.start, scene.goal, scene.obstacles, **LEGACY_PARAMS)
        row = dict(scenario=number, name=scene.name, status=result.status,
                   success=result.success, iterations=result.iterations,
                   accepted_steps=len(result.path) - 1,
                   path_length=result.path_length,
                   min_clearance=(result.min_clearance
                                  if result.min_clearance != float("inf") else None),
                   collision_segment=(result.collision_segment.tolist()
                                      if result.collision_segment is not None else None))
        rows.append(row)
        with (args.out / f"{number:02d}_path.csv").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            writer.writerows(result.path)
        if not args.no_plot:
            save_path_figure(result.path, scene.obstacles, scene.start, scene.goal,
                             args.out / f"{number:02d}_{scene.name}.png",
                             f"{scene.name}: {result.status}")
        print(f"{number}: {scene.name}: {result.status}, {result.iterations} attempts")
    with (args.out / "results.json").open("w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2, allow_nan=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
