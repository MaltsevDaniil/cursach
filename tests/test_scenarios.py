from swarm_apf.geometry import segment_circle_clearance
from swarm_apf.metrics import minimum_path_clearance
import pytest
from swarm_apf.scenarios import scenarios
from swarm_apf.planner import plan_path


def test_all_legacy_scenes_have_valid_endpoints():
    data = scenarios()
    assert len(data) == 7
    for scene in data.values():
        assert all(o.distance(scene.start) > 0 and o.distance(scene.goal) > 0
                   for o in scene.obstacles), scene.name


def test_repaired_room_has_positive_door_gap():
    scene = scenarios()[7]
    assert scene.start == (3.5, 3)
    right_wall = [o for o in scene.obstacles if o.center[0] == 5]
    # Segment across the opening has strictly positive clearance.
    assert min(segment_circle_clearance((4.9, 3), (5.1, 3), o)
               for o in right_wall) > 0
    # A complete straight line from start to goal is geometrically possible.
    assert minimum_path_clearance([scene.start, scene.goal], scene.obstacles) > 0


def test_free_space_regression_reaches_goal():
    scene = scenarios()[1]
    result = plan_path(scene.start, scene.goal, scene.obstacles, step=.08,
                       max_iterations=5000, k_att=1.5, k_rep=2.5, d0=2.5,
                       k_circ=.8, goal_tolerance=.15)
    assert result.success
    assert result.iterations == 175  # legacy behavior unchanged here


@pytest.mark.parametrize("number,expected_steps", [
    (1, 175), (2, 194), (3, 195),
    (4, 124), (5, 214), (6, 216),
])
def test_legacy_regression_scenarios(number, expected_steps):
    scene = scenarios()[number]
    result = plan_path(scene.start, scene.goal, scene.obstacles, step=.08,
                       max_iterations=5000, k_att=1.5, k_rep=2.5, d0=2.5,
                       k_circ=.8, goal_tolerance=.15)
    assert result.success and result.iterations == expected_steps
    assert result.path_length == pytest.approx(expected_steps * .08)
    assert result.min_clearance > 0
