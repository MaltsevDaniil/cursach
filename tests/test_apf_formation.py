from dataclasses import replace
import numpy as np
import pytest
from swarm_apf.models import Observation, CircleObservation, NeighborObservation
from swarm_apf.multi_apf import APFController, APFParameters
from swarm_apf.formation import (FormationLayout, FormationPlan, FormationParameters,
                                FormationController, FormationTeam, slot_position)
from swarm_apf.formation_metrics import formation_errors
from swarm_apf.swarm import SwarmSimulator


def observation(**kw):
    base = Observation(0, .05, 0, (0, 0), (0, 0), (5, 0), .7, (), (), .2)
    return replace(base, **kw)


def test_a_formula_and_speed():
    ctrl = APFController()
    obs = observation(obstacles=(CircleObservation((1, 0), .3),))
    expected = .12*(1/.5-1/1.4)/.5**2
    assert ctrl.forces(obs).obstacles == pytest.approx([-expected, 0])
    assert ctrl.forces(obs).total == pytest.approx([5-expected, 0])
    assert np.linalg.norm(ctrl.command(obs)) <= .7
    assert ctrl.command(observation(goal=(.01, 0))) == pytest.approx([.2, 0])


def test_a_zero_force_goal_and_coincident_numeric_ids():
    ctrl = APFController(APFParameters(k_att=0))
    assert np.array_equal(ctrl.command(observation()), [0, 0])
    a = observation(robot_id=10, neighbors=(NeighborObservation(2, (0, 0), (0, 0)),))
    b = observation(robot_id=2, neighbors=(NeighborObservation(10, (0, 0), (0, 0)),))
    assert ctrl.forces(a).robots == pytest.approx(-ctrl.forces(b).robots)
    assert ctrl.forces(a).robots[0] == 12
    assert APFController().command(observation(goal=(0, 0))) == pytest.approx([0, 0])


@pytest.mark.parametrize("cls", [APFParameters, FormationParameters])
def test_bad_parameters(cls):
    for kwargs in [dict(softening=2), dict(k_robot=-1), dict(max_repulsion=float("nan"))]:
        with pytest.raises(ValueError):
            cls(**kwargs)


def test_layout_and_plan():
    plan = FormationPlan((0, 0), (4, 0), 1)
    line = FormationLayout((7, 2), spacing=2)
    assert slot_position(line, plan.at(2), 7) == pytest.approx([2, -1])
    assert slot_position(line, plan.at(99), 2) == pytest.approx([4, 1])
    v = FormationLayout((0, 1, 2), "V", 2)
    assert slot_position(v, plan.at(0), 2) == pytest.approx([-2, -2])
    rotated = FormationPlan((0, 0), (0, 4), 1)
    assert slot_position(line, rotated.at(0), 7) == pytest.approx([1, 0])
    with pytest.raises(ValueError):
        plan.at(-1)


def test_b_repulsion_works_at_slot():
    layout, plan = FormationLayout((0,)), FormationPlan((0, 0), (4, 0))
    ctrl = FormationController(layout, plan, 0)
    assert ctrl.command(observation()) == pytest.approx([0, 0])
    assert ctrl.command(observation(obstacles=(CircleObservation((1, 0), .3),)))[0] < 0
    with pytest.raises(ValueError):
        ctrl.command(observation(robot_id=2))


@pytest.mark.parametrize("kind,n", [("line", 5), ("V", 10)])
def test_b_real_e1_and_translation_metric(kind, n):
    layout = FormationLayout(tuple(range(n)), kind, 1.5)
    plan = FormationPlan((0, 0), (3, 0))
    starts = np.array([slot_position(layout, plan.at(0), i) for i in layout.robot_ids])
    goals = [slot_position(layout, plan.at(plan.arrival_time), i) for i in layout.robot_ids]
    metrics = formation_errors(starts+[2, 0], layout.robot_ids, layout, plan.at(0))
    assert metrics["rms"] == pytest.approx(2)
    assert metrics["shape_rms"] == pytest.approx(0)
    sim = SwarmSimulator(controller=FormationTeam(layout, plan))
    sim.reset(starts, goals)
    result = sim.run(300)
    assert result.success
    assert np.max(np.linalg.norm(result.commands, axis=2)) <= .7+1e-12


def test_a_real_e1():
    sim = SwarmSimulator(controller=APFController())
    sim.reset([[0, 0], [0, 1.5]], [[3, 0], [3, 1.5]])
    assert sim.run(200).success
