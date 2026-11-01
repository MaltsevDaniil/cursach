"""Headless PNG rendering; not imported by the planner/controller."""

from pathlib import Path
import numpy as np


def save_path_figure(path, obstacles, start, goal, destination, title="APF + Vortex"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path = np.asarray(path, dtype=float)
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.3)
    for obs in obstacles:
        ax.add_patch(plt.Circle(obs.center, obs.radius, color="gray", alpha=0.5))
    ax.plot(path[:, 0], path[:, 1], "b-", lw=2, label="Path")
    ax.scatter([start[0]], [start[1]], c="green", label="Start", zorder=4)
    ax.scatter([goal[0]], [goal[1]], c="red", marker="*", s=160,
               label="Goal", zorder=4)
    # Include full obstacle disks, not just their centers or visited positions.
    xs = [float(start[0]), float(goal[0]), *path[:, 0].tolist()]
    ys = [float(start[1]), float(goal[1]), *path[:, 1].tolist()]
    for obs in obstacles:
        xs += [obs.center[0] - obs.radius, obs.center[0] + obs.radius]
        ys += [obs.center[1] - obs.radius, obs.center[1] + obs.radius]
    ax.set_xlim(min(xs) - 1.0, max(xs) + 1.0)
    ax.set_ylim(min(ys) - 1.0, max(ys) + 1.0)
    ax.set(xlabel="X", ylabel="Y", title=title)
    ax.legend()
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=150, bbox_inches="tight")
    plt.close(fig)
