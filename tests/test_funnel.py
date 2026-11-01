from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from swarm_apf.funnel_demo import scene,audit
from swarm_apf.models import SwarmConfig
from swarm_apf.swarm import SwarmSimulator
from swarm_apf.sensing import observe
from swarm_apf.controller_team import ControllerTeam


@pytest.mark.parametrize("n",[5,10])
def test_funnel_geometry_and_static_reference(n):
    layout,plan,start,goals,obstacles=scene(n)
    sim=SwarmSimulator()
    sim.reset(start,goals,obstacles)
    assert plan.start_anchor==plan.group_goal
    assert plan.at(0)==plan.at(10000)
    obs=observe(sim.state,0,goals,obstacles,SwarmConfig())
    c=ControllerTeam("C",layout,plan)
    assert np.array_equal(c.command(obs),c.command(replace(obs,time=1000)))


def test_external_bypass_is_not_throat_success():
    result=SimpleNamespace(robot_ids=(0,),positions=np.array([[[2.,2.]],[[4.,2.]]]),
                           goals=np.array([[4.,2.]]),config=SwarmConfig(),status="SUCCESS")
    assert audit(result)["arrived"]==1
    assert not audit(result)["all_crossed_throat"]
    result.positions=np.array([[[2.,0.]],[[4.,0.]]])
    assert audit(result)["all_crossed_throat"]
