import numpy as np
import pytest
from swarm_apf.tunnel_demo import TunnelSchedule, run, audit


def test_schedule_restores_original_shape_and_order():
    s=TunnelSchedule(10)
    assert np.allclose(s.shapes[-1]-s.shapes[-1].mean(axis=0),
                       s.shapes[0]-s.shapes[0].mean(axis=0))
    for i in range(s.n):
        for t,shape in zip(s.ends,s.shapes):
            assert s.slot(t,i)==pytest.approx(shape[i])


def test_all_robots_use_tunnel_and_reform():
    s,sim,result=run(5)
    assert result.success
    assert not result.events
    evidence=audit(result,s)
    assert evidence["all_robots_used_same_tunnel"]
    assert evidence["final_shape_rms"] < .12
    assert result.min_robot_clearance > 0
    assert result.min_obstacle_clearance > 0
