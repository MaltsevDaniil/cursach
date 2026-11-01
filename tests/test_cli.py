import json
from swarm_apf.cli import main


def test_cli_results_are_reproducible_and_record_geometry(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    assert main(["--scenario", "1", "--out", str(a), "--no-plot"]) == 0
    assert main(["--scenario", "1", "--out", str(b), "--no-plot"]) == 0
    assert (a / "results.json").read_bytes() == (b / "results.json").read_bytes()
    assert (a / "01_path.csv").read_bytes() == (b / "01_path.csv").read_bytes()
    config = json.loads((a / "config.json").read_text())
    assert config["scenarios"]["1"]["start"] == [0, 0]
    assert config["uses_randomness"] is False
