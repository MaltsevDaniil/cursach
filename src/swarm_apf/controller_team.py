"""Per-ID C/D instances and explicit per-run lifecycle, no global observations."""
from dataclasses import asdict
import csv
from pathlib import Path
from .formation import slot_position, B_VERSION
from .vortex_c import FormationVortexController, VortexCParameters
from .adaptive_d import AdaptiveDController, ABLATIONS


class ControllerTeam:
    def __init__(self, method, layout, plan, *, params=None, ablation="full"):
        if method not in ("C", "D"):
            raise ValueError("method must be C or D")
        if ablation not in ABLATIONS or (method == "C" and ablation != "full"):
            raise ValueError("ablation applies to D only")
        self.method, self.layout, self.plan = method, layout, plan
        params = params if params is not None else VortexCParameters()
        self.controllers = {}
        for ident in layout.robot_ids:
            if method == "C":
                ctrl = FormationVortexController(layout, plan, ident, params, slot_position)
            else:
                ctrl = AdaptiveDController(layout, plan, ident, base_params=params,
                                           ablation=ABLATIONS[ablation], slot_provider=slot_position)
            self.controllers[ident] = ctrl

    def reset(self):
        for ctrl in self.controllers.values():
            if isinstance(ctrl, AdaptiveDController):
                ctrl.reset()

    def command(self, obs):
        return self.controllers[obs.robot_id].command(obs)

    def metadata(self):
        return dict(method=self.method, layout=asdict(self.layout), plan=asdict(self.plan),
                    slot_implementation=B_VERSION,
                    controllers={str(i): c.metadata() for i, c in self.controllers.items()})

    def save_records(self, destination, result):
        """Logs include rejected proposals; acceptance comes from the simulator."""
        if self.method != "D":
            return
        rows = []
        for ctrl in self.controllers.values():
            for record in ctrl.records:
                row = record.as_dict()
                row["accepted"] = any(abs(record.time-t) < 1e-10 for t in result.times[:-1])
                rows.append(row)
        rows.sort(key=lambda row: (row["time"], row["robot_id"]))
        from .adaptive_d import ModeRecord
        fields = list(ModeRecord.__dataclass_fields__) + ["accepted"]
        target = Path(destination)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", newline="", encoding="utf8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
