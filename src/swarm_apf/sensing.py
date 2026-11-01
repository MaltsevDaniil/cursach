"""The ONLY module allowed to turn global WorldState into local observations."""
from __future__ import annotations

import numpy as np
from .geometry import Obstacle
from .models import (CircleObservation, NeighborObservation, Observation,
                     SwarmConfig, WorldState, vec2)


def observe(world: WorldState, index: int, goals, obstacles: tuple[Obstacle, ...],
            cfg: SwarmConfig) -> Observation:
    """Use full world *only* for filtering; never leak remote positions.

    Neighbors: center distance <= neighbor_radius. Obstacles: signed surface
    clearance <= obstacle_sensor_radius. Both tests include the boundary.
    """
    if not 0 <= index < len(world.robot_ids):
        raise IndexError(index)
    p = world.positions[index]
    v = world.velocities[index]
    neighbors = []
    for j, ident in enumerate(world.robot_ids):
        if index == j:
            continue
        delta = world.positions[j] - p
        if float(np.linalg.norm(delta)) <= cfg.neighbor_radius:
            neighbors.append(NeighborObservation(
                ident, vec2(delta), vec2(world.velocities[j] - v)))
    neighbors.sort(key=lambda n: n.robot_id)

    circles = []
    for obs in obstacles:
        delta = np.asarray(obs.center) - p
        gap = float(np.linalg.norm(delta)) - obs.radius - cfg.robot_radius
        if gap <= cfg.obstacle_sensor_radius:
            circles.append(CircleObservation(vec2(delta), obs.radius))
    circles.sort(key=lambda o: (o.relative_center, o.radius))

    return Observation(time=world.time, dt=cfg.dt, robot_id=world.robot_ids[index],
                       self_position=vec2(p), self_velocity=vec2(v),
                       goal=vec2(goals[index]), max_speed=cfg.max_speed,
                       neighbors=tuple(neighbors), obstacles=tuple(circles),
                       robot_radius=cfg.robot_radius)
