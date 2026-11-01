from dataclasses import replace
import csv
import numpy as np
import pytest
from swarm_apf.models import Observation, CircleObservation
from swarm_apf.formation import FormationLayout, FormationPlan, FormationController, slot_position
from swarm_apf.vortex_c import FormationVortexController, VortexCParameters, slot_from_b_contract
from swarm_apf.adaptive_d import AdaptiveDController, AdaptiveDParameters, Mode, ABLATIONS
from swarm_apf.controller_team import ControllerTeam
from swarm_apf.swarm import SwarmSimulator


def obs(t=0, circles=(), ident=0):
    return Observation(t, .05, ident, (0, 0), (0, 0), (4, 0), .7, (), circles, .2)


@pytest.mark.parametrize("kind", ["line", "V"])
def test_c_slot_contract_matches_reconstructed_b(kind):
    layout = FormationLayout((10, 2, 8, 4), kind)
    plan = FormationPlan((1, 2), (3, 4))
    for t in (0, 1, 100):
        for ident in layout.robot_ids:
            assert slot_position(layout, plan.at(t), ident) == pytest.approx(
                slot_from_b_contract(layout, plan.at(t), ident))


def test_c_zero_circulation_matches_b_and_fixed_sign():
    layout, plan = FormationLayout((0,)), FormationPlan((0, 0), (4, 0))
    observation = obs(1, (CircleObservation((1, 0), .3),))
    b = FormationController(layout, plan, 0)
    c = FormationVortexController(layout, plan, 0, VortexCParameters(k_circ=0), slot_position)
    assert c.command(observation) == pytest.approx(b.command(observation))
    plus = FormationVortexController(layout, plan, 0, slot_provider=slot_position)
    minus = replace(plus, params=VortexCParameters(circulation_sign=-1))
    assert plus.forces(observation).obstacles_tangential == pytest.approx(
        -minus.forces(observation).obstacles_tangential)
    assert plus.forces(observation).obstacles_tangential[1] < 0


def test_d_passage_recovery_reset_idempotence_and_ablation():
    layout, plan = FormationLayout((0,)), FormationPlan((0, 0), (4, 0))
    params = AdaptiveDParameters(min_dwell=.05, enter_samples=1, stall_samples=1,
                                 stall_window=.2)
    d = AdaptiveDController(layout, plan, 0, adaptive_params=params, slot_provider=slot_position)
    near = (CircleObservation((1, 0), .3),)
    d.command(obs(0, near))
    command = d.command(obs(.05, near))
    assert d.mode is Mode.PASSAGE
    assert d.command(obs(.05, near)) == pytest.approx(command)
    assert len(d.records) == 2
    d.command(obs(.25, near))
    assert d.mode is Mode.RECOVERY
    assert d.records[-1].w_formation == pytest.approx(.1)
    with pytest.raises(ValueError):
        d.command(obs(0))
    d.reset()
    assert d.mode is Mode.NORMAL and not d.records
    no_vortex = AdaptiveDController(layout, plan, 0, ablation=ABLATIONS["no_vortex"])
    no_vortex.command(obs(0, near))
    assert no_vortex.records[-1].k_circ == 0


@pytest.mark.parametrize("method", ["C", "D"])
def test_team_e1_repeat_run_and_isolated_memory(method, tmp_path):
    layout, plan = FormationLayout(tuple(range(5)), spacing=1.5), FormationPlan((0, 0), (3, 0))
    team = ControllerTeam(method, layout, plan)
    starts = [slot_position(layout, plan.at(0), i) for i in layout.robot_ids]
    goals = [slot_position(layout, plan.at(plan.arrival_time), i) for i in layout.robot_ids]
    sim = SwarmSimulator(controller=team)
    sim.reset(starts, goals)
    first = sim.run(300)
    assert first.success
    if method == "D":
        assert team.controllers[0]._recent is not team.controllers[1]._recent
        team.save_records(tmp_path/"modes.csv", first)
        rows = list(csv.DictReader((tmp_path/"modes.csv").open()))
        assert len(rows) == 5*first.attempted_steps
        assert all(row["accepted"] == "True" for row in rows)
    sim.reset(starts, goals)
    second = sim.run(300)
    assert np.array_equal(first.positions, second.positions)
