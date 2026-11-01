"""Versioned E2 reconstruction. Undocumented cluster/U coordinates are new."""
from collections import deque
from functools import lru_cache
import math
import numpy as np
from .geometry import Obstacle, segment_circle_clearance
from .formation import FormationLayout, FormationPlan, slot_position, B_VERSION
from .swarm import SwarmSimulator
from .experiment_runner import Case, sha

VERSION = "SCENARIO-E2-RECONSTRUCTED-1.0.0"
SCENARIOS = ("free", "single_obstacle", "obstacle_cluster", "narrow_corridor",
             "u_trap", "room_exit", "two_obstacles", "blocked_room")


def cohort(name):
    return "stress" if name == "blocked_room" else "diagnostic" if name == "two_obstacles" else "benchmark"


def obstacles_for(name):
    def row(a, b, radius=.55):
        count = int(math.ceil(np.linalg.norm(np.asarray(b)-a)/.8))+1
        return [Obstacle(tuple(p), radius) for p in np.linspace(a, b, count)]
    if name == "free":
        return ()
    if name == "single_obstacle":
        return (Obstacle((0, 0), 1),)
    if name == "two_obstacles":
        return (Obstacle((0, -1), .7), Obstacle((0, 1), .7))
    if name == "narrow_corridor":
        return tuple(row((-4, -.95), (4, -.95))+row((-4, .95), (4, .95)))
    if name == "obstacle_cluster":
        return tuple(Obstacle(c, r) for c, r in [
            ((-2, -2), .7), ((-1, 1.8), .65), ((.5, -.5), .8),
            ((2, 2), .7), ((3, -2), .6)])
    if name == "u_trap":
        return tuple(row((-3, -3), (2, -3))+row((2, -3), (2, 3))+row((2, 3), (-3, 3)))
    if name in ("room_exit", "blocked_room"):
        wall = row((-18, -8), (0, -8))+row((-18, 8), (0, 8))+row((-18, -8), (-18, 8))
        wall += (row((0, -8), (0, 8)) if name == "blocked_room" else
                 row((0, -8), (0, -1))+row((0, 1), (0, 8)))
        return tuple(wall)
    raise ValueError(name)


@lru_cache(maxsize=32)
def grid_components(obstacles, radius, epsilon, step=.1):
    """Conservative cells: inflated disks contain each cell's full square."""
    xs, ys = np.arange(-22, 16+step/2, step), np.arange(-12, 12+step/2, step)
    x, y = np.meshgrid(xs, ys)
    free = np.ones(x.shape, dtype=bool)
    for cx, cy, r in obstacles:
        free &= (x-cx)**2+(y-cy)**2 > (r+radius+step/math.sqrt(2)+1e-6+epsilon)**2
    labels = np.zeros(free.shape, dtype=np.int32)
    height, width = free.shape
    component = 0
    for iy, ix in zip(*np.nonzero(free)):
        if labels[iy, ix]:
            continue
        component += 1
        labels[iy, ix] = component
        queue = deque([(iy, ix)])
        while queue:
            cy, cx = queue.popleft()
            for ny, nx in ((cy-1,cx),(cy+1,cx),(cy,cx-1),(cy,cx+1)):
                if 0 <= ny < height and 0 <= nx < width and free[ny,nx] and not labels[ny,nx]:
                    labels[ny,nx] = component
                    queue.append((ny,nx))
    return xs, ys, labels


def audit_routes(initial, goals, obstacles, cfg):
    key = tuple((float(o.center[0]), float(o.center[1]), o.radius) for o in obstacles)
    xs, ys, labels = grid_components(key, cfg.robot_radius, cfg.epsilon_geom)
    def endpoint(p):
        ix, iy = int(round((p[0]-xs[0])/.1)), int(round((p[1]-ys[0])/.1))
        if not (0 <= ix < len(xs) and 0 <= iy < len(ys)):
            return 0
        point = (xs[ix], ys[iy])
        if any(segment_circle_clearance(p, point, o, cfg.robot_radius) <= cfg.epsilon_geom for o in obstacles):
            return 0
        return int(labels[iy, ix])
    routes = [bool(endpoint(a) and endpoint(a) == endpoint(b)) for a,b in zip(initial, goals)]
    return dict(all_routes=all(routes), per_robot=routes, grid_step=.1,
                bounds=[-22,16,-12,12], negative_may_be_inconclusive=True,
                scope="individual disk only; not a joint swarm plan")


def make_case(name, n, seed, cfg):
    if name not in SCENARIOS or n not in (5,10,20) or seed < 0:
        raise ValueError("E2 requires known scene, N=5/10/20, nonnegative seed")
    width = {"narrow_corridor": .8, "room_exit": .9, "two_obstacles": .6}.get(name)
    if width is not None and width <= 2*cfg.robot_radius+cfg.epsilon_geom:
        raise ValueError("SCENE_UNSUITABLE: disk does not fit declared opening")
    layout = FormationLayout(tuple(range(n)), "line" if n == 5 else "V", .65)
    plan = FormationPlan((-10,0), (10,0), .5)
    starts = np.array([slot_position(layout, plan.at(0), i) for i in layout.robot_ids])
    goals = np.array([slot_position(layout, plan.at(plan.arrival_time), i) for i in layout.robot_ids])
    obstacles = obstacles_for(name)
    rng = np.random.default_rng(np.random.SeedSequence([seed,n,SCENARIOS.index(name)+1,189173]))
    sim = SwarmSimulator(cfg)
    for rejected in range(64):
        initial = starts+rng.uniform(-.015,.015, starts.shape)
        try:
            sim.reset(initial, goals, obstacles)
            break
        except ValueError:
            if rejected == 63:
                raise
    audit = audit_routes(initial, goals, obstacles, cfg)
    if not audit["all_routes"] and cohort(name) != "stress":
        raise ValueError(f"SCENE_UNCERTIFIED: {name} N={n}")
    from dataclasses import asdict
    desc = dict(version=VERSION, scenario=name, n=n, seed=seed, cohort=cohort(name),
                positions=initial.tolist(), goals=goals.tolist(), physics=asdict(cfg),
                obstacles=[dict(center=o.center, radius=o.radius) for o in obstacles],
                layout=asdict(layout), plan=asdict(plan), formation_source=B_VERSION,
                rejected_placement_attempts=rejected, audit=audit,
                opening_width=width, outer_bypass_allowed=name=="narrow_corridor",
                provenance="reconstructed; cluster and U coordinates newly specified; room left wall moved to -18 to fit N=20 V slots")
    return Case(name,n,seed,layout,plan,slot_position,B_VERSION,initial,goals,obstacles,desc,sha(desc))
