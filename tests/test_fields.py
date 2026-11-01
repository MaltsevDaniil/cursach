import numpy as np
import pytest
from swarm_apf.geometry import Obstacle
from swarm_apf.fields import attractive_force, vortex_repulsive_force, total_force


def test_attraction_and_no_obstacle_force_far_away():
    assert np.allclose(attractive_force((0, 0), (2, 0), 1.5), (3, 0))
    obs = Obstacle((0, 0), 1)
    assert np.array_equal(vortex_repulsive_force((4, 0), [obs]), [0, 0])
    assert np.allclose(total_force((0, 0), (2, 0), [], k_att=1.5), [3, 0])


def test_vortex_rotates_counterclockwise_and_radial_only_when_zero():
    obs = Obstacle((0, 0), 1)
    force = vortex_repulsive_force((2, 0), [obs], k_circ=.8)
    assert force[0] > 0
    assert force[1] == pytest.approx(.8 * force[0])
    radial = vortex_repulsive_force((2, 0), [obs], k_circ=0)
    assert radial[0] > 0
    assert radial[1] == pytest.approx(0)


def test_near_surface_and_center_are_finite_and_not_dead():
    obs = Obstacle((0, 0), 1)
    for d in [2e-6, 0.5e-6, 0, -1e-7, -1]:
        f = vortex_repulsive_force((1 + d, 0), [obs])
        assert np.all(np.isfinite(f))
        assert f[0] > 0
    center = vortex_repulsive_force((0, 0), [obs])
    assert np.all(np.isfinite(center))
    assert center[0] > 0


def test_symmetric_radial_forces_cancel():
    obstacles = [Obstacle((-2, 0), .5), Obstacle((2, 0), .5)]
    force = vortex_repulsive_force((0, 0), obstacles, k_circ=0)
    assert np.allclose(force, [0, 0])


def test_invalid_force_parameters():
    with pytest.raises(ValueError):
        vortex_repulsive_force((0, 0), [], d0=0)
    with pytest.raises(ValueError):
        attractive_force((0, 0), (1, 0), k_att=-1)
