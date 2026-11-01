"""Report measured outcomes separately for benchmark/diagnostic/stress cohorts."""
import argparse
import csv
import json
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    args = parser.parse_args(argv)
    manifest = json.loads((args.input/"manifest.json").read_text())
    if manifest["run_status"] != "COMPLETE":
        raise ValueError("report requires a COMPLETE benchmark")
    if args.out.exists():
        raise FileExistsError(args.out)
    groups = []
    for prefix in ("","diagnostic_","stress_"):
        path = args.input/(prefix+"aggregate.csv")
        if path.exists():
            with path.open(newline="",encoding="utf8") as stream:
                groups.append((prefix,list(csv.DictReader(stream))))
    if not groups:
        raise ValueError("no aggregate tables")
    args.out.mkdir(parents=True)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    lines = ["# Experiment report", "", f"Source fingerprint: {manifest['code']['fingerprint']}", "",
             "A/B are reconstructed. Small samples do not establish superiority.",
             "Completion times are success-only; collisions and timeouts are separate outcomes.", ""]
    for prefix,rows in groups:
        lines += [f"## {prefix.rstrip('_') or 'benchmark'}", "",
                  "| Method | Scenario | N | Runs | Successes | Timeouts | Collisions | Invalid |",
                  "|---|---|---:|---:|---:|---:|---:|---:|"]
        for row in rows:
            lines.append("| "+" | ".join(row[k] for k in (
                "method","scenario","n","runs_total","successes","timeouts","collision_runs","runs_invalid"))+" |")
        fig,ax = plt.subplots(figsize=(max(7,len(rows)*.35),5))
        bottom = [0]*len(rows)
        for key,label in (("successes","SUCCESS"),("timeouts","TIMEOUT"),("collision_runs","COLLISION"),("runs_invalid","INVALID")):
            counts = [int(row[key]) for row in rows]
            ax.bar(range(len(rows)),counts,bottom=bottom,label=label)
            bottom = [a+b for a,b in zip(bottom,counts)]
        ax.set_xticks(range(len(rows)),[f"{r['method']} {r['scenario']} N{r['n']}" for r in rows],rotation=90)
        ax.set_ylabel("Number of runs")
        ax.legend()
        fig.tight_layout()
        filename = prefix+"outcomes.png"
        fig.savefig(args.out/filename,dpi=130)
        plt.close(fig)
        lines += ["",f"![Outcomes]({filename})",""]
    (args.out/"report.md").write_text("\n".join(lines)+"\n",encoding="utf8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
