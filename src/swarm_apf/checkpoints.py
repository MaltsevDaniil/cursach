"""Transactional run directories and verified checkpoints for E2 continuation."""
import hashlib
import json
import os
from pathlib import Path
import uuid
from . import experiment_runner as runner


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary = path.with_name(path.name+".tmp-"+uuid.uuid4().hex)
    temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf8")
    os.replace(temporary,path)


def canonical_settings(settings):
    return {k:v for k,v in settings.items() if k not in ("resume","out","config_json")}


def check_manifest(manifest, settings, code, version):
    if manifest.get("checkpoint_schema") != 1:
        raise ValueError("resume requires checkpoint schema 1; old outputs cannot be resumed")
    if manifest["version"] != version or manifest["code"]["fingerprint"] != code["fingerprint"]:
        raise ValueError("resume source/version mismatch")
    if canonical_settings(manifest["settings"]) != canonical_settings(settings):
        raise ValueError("resume settings mismatch")


def file_hashes(folder):
    return {str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.rglob("*")) if p.is_file() and p.name != "checkpoint.json"}


def run_or_load(method, case, cfg, *, max_steps, allow_proxies, target, code, resume):
    target = Path(target)
    identity = dict(method=method,scene_hash=case.scene_hash,
                    code_fingerprint=code["fingerprint"],max_steps=max_steps)
    if target.exists():
        if not resume:
            raise FileExistsError(target)
        saved = json.loads((target/"checkpoint.json").read_text(encoding="utf8"))
        if saved["identity"] != identity or saved["files"] != file_hashes(target):
            raise ValueError(f"checkpoint mismatch or corrupted files: {target}")
        return saved["row"]
    target.parent.mkdir(parents=True,exist_ok=True)
    # Partial attempts remain available for inspection and are never treated as completed.
    temporary = target.with_name(".partial-"+target.name+"-"+uuid.uuid4().hex)
    row = runner.run_case(method,case,cfg,max_steps=max_steps,allow_proxies=allow_proxies,
                          target=temporary,code=code)
    atomic_json(temporary/"checkpoint.json",dict(identity=identity,row=row,files=file_hashes(temporary)))
    temporary.rename(target)
    return row
