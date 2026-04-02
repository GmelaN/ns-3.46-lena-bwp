#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


METHODS = [
    ("rqrdqn_pf", "RQR-DQN+PF", "rl"),
    ("drqn_pf", "DRQN+PF", "rl"),
    ("drqn_aeq", "DRQN+Aequitas", "rl"),
    ("bwp0_aeq", "BWP0+Aequitas", "baseline"),
    ("bwp1_aeq", "BWP1+Aequitas", "baseline"),
]


def _load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def write_train_reward_bins(run_dir: Path, scenario: str, method_key: str, method_name: str, out_csv: Path) -> None:
    bin_path = run_dir / "train_reward_bins_300.csv"
    if not bin_path.exists():
        return
    with bin_path.open(encoding="utf-8", newline="") as f_in, out_csv.open("a", encoding="utf-8", newline="") as f_out:
        reader = csv.DictReader(f_in)
        writer = csv.writer(f_out)
        for row in reader:
            writer.writerow(
                [
                    scenario,
                    method_name,
                    method_key,
                    int(row["episode"]),
                    int(row["bin_index"]),
                    int(row["bin_size"]),
                    float(row["avg_reward"]),
                ]
            )


def write_eval_summary(root: Path, out_csv: Path) -> None:
    scenarios = [p.name for p in root.iterdir() if p.is_dir()]
    with out_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "scenario",
                "method",
                "method_key",
                "metric",
                "min",
                "p25",
                "p50",
                "p75",
                "max",
                "mean",
                "source_json",
            ]
        )
        for scenario in scenarios:
            scen_root = root / scenario
            for method_key, method_name, kind in METHODS:
                if kind == "rl":
                    data = _load_json(scen_root / "eval" / f"{method_key}_eval.json")
                    mean_thr = float(data.get("episode_metric_last", {}).get("mean_thr_mbps", {}).get("mean", 0.0))
                    mean_aoi = float(data.get("episode_metric_last", {}).get("mean_aoi_ms", {}).get("mean", 0.0))
                    pct = data.get("per_ue_last_percentiles", {})
                    src = scen_root / "eval" / f"{method_key}_eval.json"
                else:
                    data = _load_json(scen_root / "baselines" / f"{method_key}_eval.json")
                    mean_thr = float(data.get("mean_thr_mbps", 0.0))
                    mean_aoi = float(data.get("mean_aoi_ms", 0.0))
                    pct = data.get("per_ue_last_percentiles", {})
                    src = scen_root / "baselines" / f"{method_key}_eval.json"
                for metric in ("thr_mbps", "aoi_ms"):
                    vals = pct.get(metric, {})
                    writer.writerow(
                        [
                            scenario,
                            method_name,
                            method_key,
                            metric,
                            float(vals.get("min", 0.0)),
                            float(vals.get("p25", 0.0)),
                            float(vals.get("p50", 0.0)),
                            float(vals.get("p75", 0.0)),
                            float(vals.get("max", 0.0)),
                            mean_thr if metric == "thr_mbps" else mean_aoi,
                            str(src.resolve()),
                        ]
                    )


def write_train_episode_summary(root: Path, out_csv: Path) -> None:
    scenarios = [p.name for p in root.iterdir() if p.is_dir()]
    with out_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["scenario", "method", "method_key", "episode", "global_step", "mean_reward", "mean_thr_mbps", "mean_aoi_ms", "source_csv"])
        for scenario in scenarios:
            scen_root = root / scenario
            for method_key, method_name, kind in METHODS:
                if kind != "rl":
                    continue
                src = scen_root / f"{method_key}_train" / "train_episode_metrics.csv"
                with src.open(encoding="utf-8", newline="") as src_f:
                    reader = csv.DictReader(src_f)
                    for row in reader:
                        writer.writerow(
                            [
                                scenario,
                                method_name,
                                method_key,
                                int(row["episode"]),
                                int(row["global_step"]),
                                float(row["mean_reward"]),
                                float(row["mean_thr_mbps"]),
                                float(row["mean_aoi_ms"]),
                                str(src.resolve()),
                            ]
                        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    args = parser.parse_args()

    root = Path(args.root).resolve()
    scenarios = [p.name for p in root.iterdir() if p.is_dir()]
    train_bins_csv = root / "train_reward_300step_avg.csv"
    with train_bins_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["scenario", "method", "method_key", "episode", "bin_index", "bin_size", "avg_reward"])
    for scenario in scenarios:
        scen_root = root / scenario
        for method_key, method_name, kind in METHODS:
            if kind != "rl":
                continue
            write_train_reward_bins(scen_root / f"{method_key}_train", scenario, method_key, method_name, train_bins_csv)

    write_train_episode_summary(root, root / "train_episode_metrics_all.csv")
    write_eval_summary(root, root / "eval_percentiles.csv")


if __name__ == "__main__":
    main()
