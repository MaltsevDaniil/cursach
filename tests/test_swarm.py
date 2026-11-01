import json
import numpy as np
import pytest
from swarm_apf.models import SwarmConfig, WorldState
from swarm_apf.sensing import observe
from swarm_apf.geometry import Obstacle
from swarm_apf.swarm import SwarmSimulator
from swarm_apf.boids import BoidsController
from swarm_apf.swarm_cli import main


def test_snapshot_cannot_be_modified():
    source = np.array([[0., 0.]])
    world = WorldState((0,), source, source)
    source[:] = 10
    assert np.array_equal(world.positions, [[0, 0]])
    with pytest.raises(ValueError):
        world.positions.setflags(write=True)


def test_local_sensor_boundary_and_physical_radius():
    cfg = SwarmConfig(robot_radius=.3, neighbor_radius=2, obstacle_sensor_radius=1)
    world = WorldState((0, 1, 2), [[0, 0], [2, 0], [3, 0]], np.zeros((3, 2)))
    obs = observe(world, 0, [[5, 0]]*3,
                  (Obstacle((1.5, 0), .2), Obstacle((10, 0), 1)), cfg)
    assert [n.robot_id for n in obs.neighbors] == [1]
    assert len(obs.obstacles) == 1
    assert obs.robot_radius == .3
    assert not hasattr(obs, "world")


def test_crossing_disks_reject_entire_step_and_export(tmp_path):
    sim = SwarmSimulator(SwarmConfig(dt=1, max_speed=4))
    starts = [[-1, 0], [1, 0], [0, 4]]
    sim.reset(starts, [[1, 0], [-1, 0], [2, 4]])
    result = sim.run(1)
    assert result.status == "COLLISION_ROBOT"
    assert result.accepted_steps == 0 and result.attempted_steps == 1
    assert np.array_equal(sim.state.positions, starts)
    assert result.events[0].fraction == pytest.approx(.4)
    assert result.rejected_step is not None
    sim.save_history(tmp_path)
    saved = json.loads((tmp_path / "summary.json").read_text())
    assert saved["rejected_step"] is not None
    assert saved["min_obstacle_clearance"] is None


def test_unseen_obstacle_still_detected_by_physics():
    sim = SwarmSimulator(SwarmConfig(dt=1, max_speed=10, obstacle_sensor_radius=.1))
    sim.reset([[-2, 0]], [[2, 0]], [Obstacle((0, 0), .5)])
    result = sim.run(1)
    assert result.status == "COLLISION_OBSTACLE"
    assert result.accepted_steps == 0


@pytest.mark.parametrize("command", [[float("nan"), 0], [20, 0], [0, 0, 0]])
def test_invalid_commands_are_not_clipped(command):
    class Bad:
        def command(self, obs):
            return command
    sim = SwarmSimulator(controller=Bad())
    sim.reset([[0, 0]], [[5, 0]])
    assert sim.run(1).status == "NUMERIC_ERROR"
    assert sim.state.time == 0


def test_order_independence_and_boids_real_observations():
    a, b = SwarmSimulator(controller=BoidsController()), SwarmSimulator(controller=BoidsController())
    p, g = np.array([[0, 0], [0, 1.5]]), np.array([[4, 0], [4, 1.5]])
    a.reset(p, g, robot_ids=[8, 3])
    b.reset(p[::-1], g[::-1], robot_ids=[3, 8])
    ra, rb = a.run(20), b.run(20)
    assert ra.status != "NUMERIC_ERROR"
    assert np.array_equal(ra.positions, rb.positions)


def test_all_observations_use_previous_snapshot():
    seen = []
    class Record:
        def command(self, obs):
            seen.append(obs)
            return np.array([.5, 0])
    sim = SwarmSimulator(controller=Record())
    sim.reset([[0, 0], [1, 0]], [[5, 0], [6, 0]])
    sim.step()
    assert seen[0].time == seen[1].time == 0
    assert seen[1].neighbors[0].relative_position == (-1., 0.)
    assert seen[1].neighbors[0].relative_velocity == (0., 0.)


def test_failed_reset_keeps_state_and_initial_success():
    sim = SwarmSimulator()
    sim.reset([[0, 0]], [[0, 0]])
    before = sim.state
    with pytest.raises(ValueError):
        sim.reset([[0, 0], [0, 0]], [[3, 0], [4, 0]])
    assert sim.state is before
    assert sim.run().success
    assert sim.result().accepted_steps == 0


def test_cli_free_swarm(tmp_path):
    assert main(["--out", str(tmp_path), "--no-plot"]) == 0
    assert json.loads((tmp_path/"summary.json").read_text())["success"]
