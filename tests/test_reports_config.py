import json
import pytest
from swarm_apf.experiment_e2 import main
from swarm_apf.report_cli import main as report


def test_json_cli_precedence_and_boolean_override(tmp_path):
    config = tmp_path/"config.json"
    config.write_text(json.dumps(dict(out=str(tmp_path/"wrong"),scenarios=["free"],robots=[5],
                                     algorithms=["C"],max_steps=0,preflight_only=True)))
    target = tmp_path/"right"
    main(["--config-json",str(config),"--out",str(target),"--no-preflight-only"])
    assert not (tmp_path/"wrong").exists()
    assert json.loads((target/"manifest.json").read_text())["run_status"] == "COMPLETE"
    report(["--input",str(target),"--out",str(tmp_path/"report")])
    assert (tmp_path/"report/outcomes.png").exists()
    assert "Timeouts" in (tmp_path/"report/report.md").read_text()


@pytest.mark.parametrize("data", [{"robots":[True]}, {"robot_radius":True},
                                  {"unknown":1}, {"robots":[]}, {"preflight_only":"yes"}])
def test_invalid_json_values_fail(data,tmp_path):
    path = tmp_path/"bad.json"
    path.write_text(json.dumps(data))
    with pytest.raises(SystemExit):
        main(["--config-json",str(path),"--out",str(tmp_path/"out")])


def test_incomplete_report_rejected(tmp_path):
    (tmp_path/"manifest.json").write_text('{"run_status":"IN_PROGRESS"}')
    with pytest.raises(ValueError,match="COMPLETE"):
        report(["--input",str(tmp_path),"--out",str(tmp_path/"report")])
