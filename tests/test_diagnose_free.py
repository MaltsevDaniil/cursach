import numpy as np
import pytest
from swarm_apf.diagnose_free import goal_forces, motion_summary
from swarm_apf.models import SwarmConfig


def test_dense_slots_have_nonzero_outer_repulsion():
    dense = np.array([[0,k*.65] for k in range(5)])
    forces = np.array(goal_forces(dense,SwarmConfig()))
    assert forces[0,1] == pytest.approx(-8.32)
    assert forces[-1,1] == pytest.approx(8.32)
    wide = np.array([[0,k*1.4] for k in range(5)])
    assert np.allclose(goal_forces(wide,SwarmConfig()),0)


def test_motion_diagnostic_distinguishes_travel_from_oscillation():
    points = np.array([[[0.,0.]],[[1.,0.]],[[0.,0.]]])
    result = motion_summary(points,[0,1,2])
    assert result["path_per_robot"] == [2]
    assert result["displacement_to_path"] == [0]
