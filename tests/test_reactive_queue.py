from dataclasses import replace
import numpy as np
import pytest
from swarm_apf.models import Observation,CircleObservation,NeighborObservation,SwarmConfig
from swarm_apf.reactive_queue import ReactiveQueueController,ReactiveQueueTeam
from swarm_apf.funnel_demo import scene,audit
from swarm_apf.swarm import SwarmSimulator


def observation():
    return Observation(0,.05,0,(-5.,0.),(0.,0.),(12.,2.),.6,(),(),.2)


def test_no_schedule_and_reset():
    a=ReactiveQueueController(0)
    b=ReactiveQueueController(0)
    o=replace(observation(),obstacles=(CircleObservation((1.,1.),.3),))
    assert a.command(o)==pytest.approx(b.command(replace(o,time=10000)))
    assert a.encountered and not a.cleared
    a.command(observation())
    assert a.cleared
    a.reset()
    assert not a.encountered and not a.cleared


def test_priority_and_speed_limit():
    o=replace(observation(),goal=(12.,0.))
    c=ReactiveQueueController(0)
    free=c.command(o)
    queued=c.command(replace(o,neighbors=(NeighborObservation(1,(.7,0.),(0.,0.)),)))
    assert queued[0] < free[0]
    assert np.linalg.norm(queued)<=o.max_speed+1e-10
    with pytest.raises(ValueError):
        ReactiveQueueController(0,heading=(0,0))


def test_five_robots_pass_throat_without_schedule():
    layout,plan,start,goals,obstacles=scene(5)
    sim=SwarmSimulator(SwarmConfig(dt=.05,max_speed=.6),
                       ReactiveQueueTeam(layout.robot_ids,plan.group_goal))
    sim.reset(start,goals,obstacles)
    result=sim.run(1800)
    assert result.success and not result.events
    assert audit(result)["all_crossed_throat"]
    assert result.min_robot_clearance>0 and result.min_obstacle_clearance>0
    assert np.max(np.linalg.norm(result.positions[-1]-goals,axis=1))<=.25
