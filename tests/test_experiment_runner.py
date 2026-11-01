import csv
import json
import numpy as np
import pytest
from swarm_apf.experiment_runner import main, make_case, SCENARIOS
from swarm_apf.models import SwarmConfig


def test_reconstructions_fail_closed(tmp_path):
    target = tmp_path/"blocked"
    with pytest.raises(RuntimeError, match="reconstructed"):
        main(["--out", str(target)])
    assert not target.exists()


def test_scenes_preflight_and_seed_reproducibility():
    cfg = SwarmConfig()
    for name in SCENARIOS:
        for n in (5, 10, 20):
            a = make_case(name, n, 0, cfg)
            b = make_case(name, n, 0, cfg)
            assert a.scene_hash == b.scene_hash
            assert np.array_equal(a.initial, b.initial)


def test_factorial_run_shared_geometry_and_replay(tmp_path):
    flags = ["--allow-contract-proxies", "--scenarios", "free", "--robots", "5",
             "--seeds", "0", "--max-steps", "3"]
    a, b = tmp_path/"a", tmp_path/"b"
    for target in (a, b):
        assert main(flags+["--out", str(target)]) == 0
    rows = list(csv.DictReader((a/"runs.csv").open()))
    assert len(rows) == 4
    assert len({r["scene_hash"] for r in rows}) == 1
    assert all(r["status"] == "TIMEOUT" for r in rows)
    assert rows[0]["controller_source"] == "contract_reconstruction_NOT_original_A"
    assert "RECONSTRUCTED" in rows[0]["formation_source"]
    assert json.loads((a/"manifest.json").read_text())["run_status"] == "COMPLETE"
    for method in "ABCD":
        path = f"runs/{method}/free/N5/seed_0000/history.csv"
        assert (a/path).read_bytes() == (b/path).read_bytes()
    with pytest.raises(FileExistsError):
        main(flags+["--out", str(a)])


def test_zero_steps_d_and_fingerprint_guard(tmp_path):
    flags = ["--algorithms", "D", "--scenarios", "free", "--robots", "5",
             "--seeds", "0", "--max-steps", "0"]
    with pytest.raises(RuntimeError, match="fingerprint"):
        main(flags+["--expect-code-sha", "wrong", "--out", str(tmp_path/"blocked")])
    assert main(flags+["--out", str(tmp_path/"zero")]) == 0
    summary = json.loads((tmp_path/"zero/runs/D/free/N5/seed_0000/summary.json").read_text())
    assert summary["metrics"]["attempted_steps"] == 0
    assert summary["metrics"]["completion_time_s"] is None
