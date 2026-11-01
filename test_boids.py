"""Contract tests: synthetic public E1 observations, no global WorldState.

Full E1 integration / collision status tests must be run in the real E1 tree.
"""

from dataclasses import dataclass, field
from types import SimpleNamespace as NS
import numpy as np
import pytest

from swarm_apf.boids import BoidsController, BoidsParameters, BOIDS_VERSION


def observed(*, position=(0, 0), goal=(10, 0), velocity=(0, 0),
             neighbors=(), obstacles=(), robot_id=1, dt=0.1, radius=0.2,
             max_speed=1.0):
    return NS(self_position=position, goal=goal, self_velocity=velocity,
              neighbors=neighbors, obstacles=obstacles, robot_id=robot_id,
              robot_radius=radius, dt=dt, max_speed=max_speed)


def neighbor(position, *, velocity=(0, 0), robot_id=2):
    return NS(relative_position=position, velocity=velocity, robot_id=robot_id)


def only(**weights):
    defaults = dict(w_goal=0, w_separation=0, w_cohesion=0,
                    w_alignment=0, w_obstacle=0)
    defaults.update(weights)
    return BoidsController(BoidsParameters(**defaults))


def test_separation_uses_surface_clearance_and_coincident_id_fallback():
    ctrl = only(w_separation=1)
    close = observed(neighbors=(neighbor((0.7, 0)),))  # gap=0.3 < 0.75
    assert np.allclose(ctrl.forces(close).separation, [-0.6, 0])
    assert np.allclose(ctrl.command(close), [-1, 0])
    far = observed(neighbors=(neighbor((1.3, 0)),))  # gap=0.9 > 0.75
    assert np.array_equal(ctrl.forces(far).separation, [0, 0])
    overlap_low = observed(robot_id=1, neighbors=(neighbor((0, 0), robot_id=2),))
    overlap_high = observed(robot_id=2, neighbors=(neighbor((0, 0), robot_id=1),))
    assert np.allclose(ctrl.forces(overlap_low).separation,
                       -ctrl.forces(overlap_high).separation)


def test_cohesion_uses_only_observed_neighbors_in_local_radius():
    ctrl = only(w_cohesion=1)
    visible = observed(neighbors=(neighbor((1, 0)), neighbor((1.5, 0)),
                                  neighbor((4, 50))))
    terms = ctrl.forces(visible)
    assert np.allclose(terms.cohesion, [0.5, 0])  # (1+1.5)/(2*2.5)
    assert np.allclose(ctrl.command(visible), [1, 0])
    # A hidden robot, global centroid and other goals aren't provided at all.
    assert np.allclose(ctrl.forces(observed()).cohesion, [0, 0])


def test_alignment_uses_last_accepted_velocities_with_two_sensor_conventions():
    ctrl = only(w_alignment=1)
    abs_obs = observed(velocity=(0, 1), neighbors=(neighbor((1, 0), velocity=(1, 1)),))
    assert np.allclose(ctrl.forces(abs_obs).alignment, [1, 0])
    relative_obs = observed(velocity=(0, 1), neighbors=(NS(relative_position=(1, 0),
                                                           relative_velocity=(1, 0)),))
    assert np.allclose(ctrl.forces(relative_obs).alignment, [1, 0])
    at_start = observed(neighbors=(neighbor((1, 0)),))
    assert np.array_equal(ctrl.forces(at_start).alignment, [0, 0])
    with pytest.raises(AttributeError, match="relative_velocity"):
        ctrl.forces(observed(neighbors=(NS(relative_position=(1, 0)),)))
    # If alignment is off, a velocity-free E1 neighbor is sufficient.
    assert np.allclose(only(w_cohesion=1).forces(
        observed(neighbors=(NS(relative_position=(1, 0)),))).cohesion, [.4, 0])


def test_obstacle_avoidance_is_explicit_and_opt_in():
    obstacle = NS(relative_center=(1, 0), radius=.5)
    view = observed(obstacles=(obstacle,))  # gap = 0.3
    assert np.array_equal(only(w_goal=1).forces(view).obstacle, [0, 0])
    ctrl = only(w_obstacle=1)
    assert np.allclose(ctrl.forces(view).obstacle, [-.75, 0])
    assert np.allclose(ctrl.command(view), [-1, 0])
    with pytest.raises(ValueError):
        BoidsParameters(interaction_radius=0)
    with pytest.raises(ValueError):
        BoidsParameters(w_alignment=float("nan"))


def test_goal_speed_bounds_determinism_and_no_collision_guarantee():
    ctrl = BoidsController()
    view = observed(goal=(0.03, 0), dt=0.1, max_speed=.8)
    assert np.linalg.norm(ctrl.command(view)) <= .3 + 1e-12
    assert np.array_equal(ctrl.command(observed(goal=(0, 0))), [0, 0])
    a = observed(neighbors=(neighbor((.7, 0)), neighbor((1, 0), robot_id=3)))
    b = observed(neighbors=tuple(reversed(a.neighbors)))
    assert np.allclose(ctrl.command(a), ctrl.command(b))
    assert ctrl.metadata()["version"] == BOIDS_VERSION

    # Predict a collision from *synchronous* goal-only commands. This test
    # never pretends to be the E1 swept-collision detector.
    goal_only = only(w_goal=1)
    left = observed(position=(-1, 0), goal=(1, 0), dt=1, max_speed=2,
                    neighbors=(neighbor((2, 0)),))
    right = observed(position=(1, 0), goal=(-1, 0), dt=1, max_speed=2,
                     neighbors=(neighbor((-2, 0)),), robot_id=2)
    starts = np.array([left.self_position, right.self_position], float)
    proposals = starts + np.array([goal_only.command(left), goal_only.command(right)])
    dr0 = starts[0] - starts[1]
    dr = (proposals[0] - starts[0]) - (proposals[1] - starts[1])
    tau = float(np.clip(-np.dot(dr0, dr) / np.dot(dr, dr), 0, 1))
    assert np.linalg.norm(dr0 + tau * dr) < 2 * left.robot_radius
