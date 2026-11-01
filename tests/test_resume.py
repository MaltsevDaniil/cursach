import json
import pytest
from swarm_apf.experiment_e2 import main
from swarm_apf import checkpoints


def flags(target):
    return ["--out",str(target),"--scenarios","free","--robots","5",
            "--algorithms","C","D","--max-steps","2"]


def test_interrupt_resume_matches_fresh(tmp_path,monkeypatch):
    target = tmp_path/"interrupted"
    original = checkpoints.runner.run_case
    calls = []
    def fail_second(method,*args,**kwargs):
        calls.append(method)
        if method == "D":
            raise RuntimeError("simulated interruption")
        return original(method,*args,**kwargs)
    monkeypatch.setattr(checkpoints.runner,"run_case",fail_second)
    with pytest.raises(RuntimeError,match="simulated"):
        main(flags(target))
    assert json.loads((target/"manifest.json").read_text())["run_status"] == "IN_PROGRESS"
    c_path=target/"runs/C/free/N5/seed_0000/history.csv"
    before=c_path.stat().st_mtime_ns
    calls.clear()
    def record(method,*args,**kwargs):
        calls.append(method)
        return original(method,*args,**kwargs)
    monkeypatch.setattr(checkpoints.runner,"run_case",record)
    main(flags(target)+["--resume"])
    assert calls == ["D"]
    assert c_path.stat().st_mtime_ns == before
    fresh=tmp_path/"fresh"
    main(flags(fresh))
    for method in "CD":
        relative=f"runs/{method}/free/N5/seed_0000/history.csv"
        assert (target/relative).read_bytes() == (fresh/relative).read_bytes()


def test_resume_rejects_settings_code_and_corruption(tmp_path):
    target=tmp_path/"run"
    main(flags(target))
    with pytest.raises(ValueError,match="settings"):
        main(flags(target)+["--resume","--max-steps","3"])
    path=target/"manifest.json"
    original=path.read_text()
    manifest=json.loads(original)
    manifest["code"]["fingerprint"]="wrong"
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match="source"):
        main(flags(target)+["--resume"])
    path.write_text(original)
    (target/"runs/C/free/N5/seed_0000/history.csv").write_text("corrupt")
    with pytest.raises(ValueError,match="corrupted"):
        main(flags(target)+["--resume"])
