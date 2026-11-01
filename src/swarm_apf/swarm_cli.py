"""Reproducible E1 diagnostics with optional Boids steering."""
import argparse
from pathlib import Path
import numpy as np
from .boids import BoidsController
from .geometry import Obstacle
from .models import SwarmConfig
from .swarm import SwarmSimulator
from .swarm_controller import GoalController
from .swarm_visualization import save_swarm_figure


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, choices=range(5, 21), default=5)
    parser.add_argument("--controller", choices=["goal", "boids"], default="goal")
    parser.add_argument("--scenario", choices=["free", "obstacle"], default="free")
    parser.add_argument("--max-steps", type=int, default=600)
    parser.add_argument("--out", type=Path, default=Path("results/swarm"))
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args(argv)
    if args.max_steps < 0:
        parser.error("--max-steps must be nonnegative")
    positions = np.column_stack([np.zeros(args.robots), np.arange(args.robots)*1.5])
    goals = positions + [6, 0]
    obstacles = (Obstacle((3, 0), .6),) if args.scenario == "obstacle" else ()
    controller = BoidsController() if args.controller == "boids" else GoalController()
    sim = SwarmSimulator(SwarmConfig(), controller)
    sim.reset(positions, goals, obstacles)
    result = sim.run(args.max_steps)
    sim.save_history(args.out)
    if not args.no_plot:
        save_swarm_figure(result, args.out / "trajectory.png")
    print(f"{result.status}: {result.accepted_steps} accepted / {result.attempted_steps} attempted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
