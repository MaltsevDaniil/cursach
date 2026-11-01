"""Offline rendering; global trajectories never enter controllers."""
from pathlib import Path
import numpy as np


def save_swarm_figure(result, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 6))
    for obs in result.obstacles:
        ax.add_patch(plt.Circle(obs.center, obs.radius, color="gray", alpha=.5))
    for i, ident in enumerate(result.robot_ids):
        path = result.positions[:, i]
        line, = ax.plot(path[:, 0], path[:, 1], label=str(ident))
        color = line.get_color()
        ax.scatter(*result.goals[i], marker="x", color=color)
        ax.add_patch(plt.Circle(path[-1], result.config.robot_radius,
                               color=color, alpha=.5))
        if result.rejected_step is not None:
            segment = np.array([result.rejected_step.start[i], result.rejected_step.proposed[i]])
            ax.plot(segment[:, 0], segment[:, 1], "r--")
    ax.set(aspect="equal", title=result.status, xlabel="x", ylabel="y")
    ax.autoscale_view()
    ax.margins(.15)
    ax.grid(alpha=.3)
    ax.legend(title="Robot ID")
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=130, bbox_inches="tight")
    plt.close(fig)
