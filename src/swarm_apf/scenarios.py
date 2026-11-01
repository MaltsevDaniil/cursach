"""Seven legacy single-point scenarios, with repaired #7 geometry."""

from dataclasses import dataclass
import numpy as np
from .geometry import Obstacle


@dataclass(frozen=True)
class Scenario:
    name: str
    start: tuple[float, float]
    goal: tuple[float, float]
    obstacles: tuple[Obstacle, ...]


def circle_line_x(xs, y, radius) -> tuple[Obstacle, ...]:
    return tuple(Obstacle((float(x), y), radius) for x in xs)


def circle_line_y(x, ys, radius) -> tuple[Obstacle, ...]:
    return tuple(Obstacle((x, float(y)), radius) for y in ys)


def scenarios() -> dict[int, Scenario]:
    """Preserve old 1–6 as regression scenes, not swarm specification S01–S06."""
    one = (Obstacle((5.0, 5.0), 1.5),)
    two = (Obstacle((4.0, 5.0), 1.2), Obstacle((6.0, 5.0), 1.2))
    corridor = circle_line_x(np.linspace(1, 9, 9), 7.0, 0.8) + \
               circle_line_x(np.linspace(1, 9, 9), 3.0, 0.8)
    u = (circle_line_x(np.linspace(2, 8, 10), 3.0, 0.7)
         + circle_line_y(2.0, np.linspace(3, 9, 12), 0.7)
         + circle_line_y(8.0, np.linspace(3, 9, 12), 0.7))
    cluster = tuple(Obstacle(c, r) for c, r in [
        ((4, 3), 1.2), ((6, 3.5), 1.0), ((3.5, 5.5), 1.0),
        ((5.5, 5), 1.3), ((4.5, 7), 1.1), ((6.5, 6.5), 1.0),
        ((7, 4), 0.9), ((5, 4), 0.8), ((2, 1), 1.0), ((8, 8), 1.0),
    ])
    # Legacy #7 started INSIDE a circle, and its doorway was sealed.
    # Starting at (3.5,3) avoids the internal disk at (2.5,3), R=.8.
    # Door disks at y=2.1 and 3.9 leave a surface gap 0.8 for a point.
    room = (circle_line_y(1, np.linspace(0, 6, 13), 0.5)
            + circle_line_x(np.linspace(1, 5, 9), 6, 0.5)
            + circle_line_x(np.linspace(1, 5, 9), 0, 0.5)
            + circle_line_y(5, np.linspace(3.9, 6, 5), 0.5)
            + circle_line_y(5, np.linspace(0, 2.1, 5), 0.5)
            + (Obstacle((2.5, 3), 0.8), Obstacle((7, 2), 0.7),
               Obstacle((8, 4), 0.7)))
    return {
        1: Scenario("free_space", (0, 0), (10, 10), ()),
        2: Scenario("single_obstacle", (0, 0), (10, 10), one),
        3: Scenario("two_obstacles", (0, 0), (10, 10), two),
        4: Scenario("narrow_corridor", (0, 5), (10, 5), corridor),
        5: Scenario("u_outline_outside_start", (5, 2), (5, 9), u),
        6: Scenario("obstacle_cluster", (0, 0), (10, 10), cluster),
        7: Scenario("room_with_open_door", (3.5, 3), (10, 3), room),
    }
