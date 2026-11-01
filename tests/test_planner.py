import numpy as np
import pytest
from swarm_apf.controller import is_stagnant
from swarm_apf.geometry import Obstacle
from swarm_apf.metrics import calculate_path_length, minimum_path_clearance
from swarm_apf.planner import plan_path


def test_already_at_goal_without_step():
    result = plan_path((0, 0), (0, 0), [])
    assert result.success and result.iterations == 0
    assert result.path.shape == (1, 2) and result.path_length == 0


def test_goal_inside_tolerance_already_succeeds():
    result = plan_path((0, 0), (.03, 0), [], step=.05, goal_tolerance=.1)
    assert result.success and result.iterations == 0


def test_no_free_space_overshoot_for_small_tolerance():
    result = plan_path((0, 0), (.04, 0), [], step=.08, goal_tolerance=.01)
    assert result.success and result.iterations == 1
    assert np.allclose(result.path[-1], [.04, 0])


def test_collision_on_segment_stops_before_invalid_step():
    result = plan_path((-2.5, 0), (2.5, 0), [Obstacle((0, 0), 1)],
                       step=5, d0=1, max_iterations=1)
    assert result.status == "COLLISION_OBSTACLE"
    assert result.iterations == 1 and len(result.path) == 1
    assert result.collision_segment is not None
    assert result.min_clearance < 0


def test_initial_collision_and_goal_collision_invalid():
    disk = [Obstacle((0, 0), 1)]
    assert plan_path((0, 0), (3, 0), disk).status == "INVALID_SCENE"
    assert plan_path((3, 0), (0, 0), disk).status == "INVALID_SCENE"
    assert plan_path((1, 0), (3, 0), disk).status == "INVALID_SCENE"


def test_force_cancellation_has_no_unsafe_goal_fallback():
    result = plan_path((0, 0), (2, 0), [], k_att=0, max_iterations=10)
    assert result.status == "ZERO_FORCE" and result.iterations == 1
    assert len(result.path) == 1


def test_stagnation_detects_oscillation_and_stop_not_slow_progress():
    oscillation = np.array([[0, 0], [.08, 0]] * 21)
    assert is_stagnant(oscillation)
    assert is_stagnant(np.zeros((41, 2)))
    slow = np.array([[i * .005, 0] for i in range(41)])
    assert not is_stagnant(slow)
    assert plan_path((0, 0), (2, 0), [], step=.005,
                     max_iterations=42, goal_tolerance=.001).status == "TIMEOUT"


def test_actual_iteration_count_and_timeout():
    result = plan_path((0, 0), (1, 0), [], max_iterations=0)
    assert result.status == "TIMEOUT" and result.iterations == 0
    assert len(result.path) == 1


def test_same_inputs_independent_of_global_rng_and_other_runs():
    kwargs = dict(step=.08, max_iterations=200, goal_tolerance=.15)
    obstacles = [Obstacle((5, 5), 1.5)]
    a = plan_path((0, 0), (10, 10), obstacles, **kwargs)
    np.random.seed(735)
    np.random.random(1000)
    plan_path((0, 0), (20, 0), [], max_iterations=20)
    b = plan_path((0, 0), (10, 10), obstacles, **kwargs)
    assert a.status == b.status
    assert a.iterations == b.iterations
    assert np.array_equal(a.path, b.path)
    assert a.path_length == b.path_length


def test_length_and_clearance_metrics():
    path = np.array([[0, 0], [3, 4]])
    assert calculate_path_length(path) == pytest.approx(5)
    assert minimum_path_clearance(path, [Obstacle((1.5, 2), .3)]) == pytest.approx(-.3)


@pytest.mark.parametrize("kwargs", [{"step": 0}, {"goal_tolerance": 0},
                                    {"max_iterations": -1}, {"d0": 0}])
def test_rejects_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        plan_path((0, 0), (1, 0), [], **kwargs)


def test_exact_force_cancellation_never_tunnels_toward_goal():
    # Goal lies beyond the disk, but its attraction cancels radial repulsion.
    gap = .75
    k_rep = 2 * (gap**2 + 1e-10) / (1 / gap - 1 / 2)
    result = plan_path((0, 0), (2, 0), [Obstacle((1, 0), .25)],
                       k_att=1, k_rep=k_rep, k_circ=0, d0=2,
                       max_iterations=10)
    assert result.status == "ZERO_FORCE"
    assert len(result.path) == 1
