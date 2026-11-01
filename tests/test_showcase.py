import numpy as np
import pytest
from PIL import Image
from swarm_apf.showcase import make_scene, animate, SCENES
from swarm_apf.swarm import SwarmSimulator


@pytest.mark.parametrize("name",SCENES)
@pytest.mark.parametrize("n",[10,20,30])
def test_showcase_geometry_and_seed(name,n):
    layout,plan,start,goals,obstacles=make_scene(name,n)
    sim=SwarmSimulator()
    sim.reset(start,goals,obstacles)
    assert np.array_equal(start,make_scene(name,n)[2])
    assert len(layout.robot_ids)==n


def test_animation_contains_last_accepted_frame(tmp_path):
    layout,plan,start,goals,obstacles=make_scene("assembly",10)
    sim=SwarmSimulator()
    sim.reset(start,goals,obstacles)
    result=sim.run(3)
    target=tmp_path/"motion.gif"
    animate(result,layout,plan,target,frames=4)
    with Image.open(target) as im:
        assert im.n_frames==4
        assert im.size==(1000,700)
