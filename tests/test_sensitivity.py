import numpy as np
import pytest
from swarm_apf.sensitivity_cli import steps_for
from swarm_apf.diagnose_free import spacing_case
from swarm_apf.scenarios_e2 import make_case
from swarm_apf.models import SwarmConfig


def test_equal_physical_budget():
    assert steps_for(80,.05) == 1600
    assert steps_for(80,.025) == 3200
    with pytest.raises(ValueError):
        steps_for(1,.3)
    with pytest.raises(ValueError):
        steps_for(float("nan"),.05)


def test_jitter_is_paired_across_dt_and_spacing():
    a,b=SwarmConfig(dt=.05),SwarmConfig(dt=.025)
    ca=spacing_case(make_case("free",5,1,a),a,1.4)
    cb=spacing_case(make_case("free",5,1,b),b,1.4)
    assert np.array_equal(ca.initial,cb.initial)
    assert np.array_equal(ca.goals,cb.goals)
    assert ca.scene_hash != cb.scene_hash
    assert ca.scene_dict["physics"]["dt"] == .05
    assert cb.scene_dict["physics"]["dt"] == .025
