import json
import pytest
from swarm_apf.scenarios_e2 import make_case
from swarm_apf.models import SwarmConfig
from swarm_apf.experiment_e2 import main


def test_door_and_closed_room():
    cfg = SwarmConfig()
    for n in (5,10,20):
        assert make_case("room_exit",n,0,cfg).scene_dict["audit"]["all_routes"]
        assert not any(make_case("blocked_room",n,0,cfg).scene_dict["audit"]["per_robot"])


def test_opening_rejects_oversized_robot():
    with pytest.raises(ValueError, match="SCENE_UNSUITABLE"):
        make_case("narrow_corridor",5,0,SwarmConfig(robot_radius=.4))


def test_preflight_has_no_runs(tmp_path):
    assert main(["--out",str(tmp_path/"audit"),"--scenarios","free",
                 "--robots","5","--preflight-only"]) == 0
    assert not (tmp_path/"audit/runs.csv").exists()
    assert json.loads((tmp_path/"audit/manifest.json").read_text())["run_status"] == "PREFLIGHT_COMPLETE"


def test_stress_is_separate_and_mixing_fails(tmp_path):
    with pytest.raises(SystemExit):
        main(["--out",str(tmp_path/"bad"),"--scenarios","free","blocked_room"])
    main(["--out",str(tmp_path/"stress"),"--scenarios","blocked_room","--stress-only",
          "--robots","5","--algorithms","D","--max-steps","0"])
    assert (tmp_path/"stress/stress_aggregate.csv").exists()
    assert not (tmp_path/"stress/aggregate.csv").exists()


def test_diagnostic_does_not_enter_main_aggregate(tmp_path):
    main(["--out",str(tmp_path/"diagnostic"),"--scenarios","two_obstacles",
          "--robots","5","--algorithms","C","--max-steps","0"])
    assert (tmp_path/"diagnostic/diagnostic_aggregate.csv").exists()
    assert not (tmp_path/"diagnostic/aggregate.csv").exists()
