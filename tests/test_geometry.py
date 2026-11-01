import numpy as np
import pytest
from swarm_apf.geometry import (Obstacle, segment_circle_clearance,
                                segment_hits_obstacle)


def test_signed_distance_and_boundary():
    obs = Obstacle((0, 0), 1)
    assert [obs.distance(p) for p in [(2, 0), (1, 0), (0, 0)]] == [1, 0, -1]
    assert not obs.contains((2, 0))
    assert obs.contains((1, 0))
    assert obs.contains((0, 0))


@pytest.mark.parametrize("center,radius", [((0,), 1), ((0, 0), 0),
                                             ((0, 0), -1), ((float("nan"), 0), 1)])
def test_invalid_obstacles(center, radius):
    with pytest.raises(ValueError):
        Obstacle(center, radius)


def test_segment_crosses_disk_even_if_endpoints_are_safe():
    obs = Obstacle((0, 0), 1)
    assert obs.distance((-2.5, 0)) > 0
    assert obs.distance((2.5, 0)) > 0
    assert segment_circle_clearance((-2.5, 0), (2.5, 0), obs) == -1
    assert segment_hits_obstacle((-2.5, 0), (2.5, 0), obs)


def test_tangent_zero_length_and_agent_radius():
    obs = Obstacle((0, 0), 1)
    assert segment_circle_clearance((-2, 1), (2, 1), obs) == pytest.approx(0)
    assert segment_hits_obstacle((-2, 1), (2, 1), obs)
    assert segment_circle_clearance((2, 0), (2, 0), obs) == pytest.approx(1)
    assert segment_circle_clearance((2, 0), (2, 0), obs, .5) == pytest.approx(.5)
    assert not segment_hits_obstacle((2, 0), (2, 0), obs)
