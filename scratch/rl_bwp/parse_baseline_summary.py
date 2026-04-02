#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os

import numpy as np


def parse_kv_line(line: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for part in line.strip().split(","):
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def resolve_aoi_ms(row: dict[str, str]) -> float:
    if "aoiNodeMs" in row:
        return float(row["aoiNodeMs"])

    # Legacy onoff summaries expose burst/background AoI separately.
    parts = []
    for key in ("aoiBurstMs", "aoiBgMs"):
        if key in row:
            parts.append(float(row[key]))
    if not parts:
        raise KeyError("No AoI field found in summary row")
    return float(sum(parts) / len(parts))


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse ns-3 summaryFile text into graph-friendly JSON.")
    parser.add_argument("--summary-file", required=True)
    parser.add_argument("--output-json", required=True)
    args = parser.parse_args()

    with open(args.summary_file, encoding="utf-8") as f:
        lines = [line.strip() for line in f if line.strip()]

    if not lines:
        raise ValueError(f"Empty summary file: {args.summary_file}")

    header = parse_kv_line(lines[0])
    ue_rows = [parse_kv_line(line) for line in lines[1:] if line.startswith("ue=")]

    thr_vals = np.asarray([float(row["thrMbps"]) for row in ue_rows], dtype=np.float64)
    aoi_vals = np.asarray([resolve_aoi_ms(row) for row in ue_rows], dtype=np.float64)

    out = {
        "summary_file": os.path.abspath(args.summary_file),
        "mean_thr_mbps": float(header.get("runMeanThrMbps", 0.0)),
        "mean_aoi_ms": float(header.get("runMeanAoiMs", 0.0)),
        "mean_switches": float(header.get("runMeanSwitchCount", 0.0)),
        "per_ue_last_percentiles": {
            "thr_mbps": {
                "min": float(np.min(thr_vals)),
                "p25": float(np.percentile(thr_vals, 25)),
                "p50": float(np.percentile(thr_vals, 50)),
                "p75": float(np.percentile(thr_vals, 75)),
                "max": float(np.max(thr_vals)),
            },
            "aoi_ms": {
                "min": float(np.min(aoi_vals)),
                "p25": float(np.percentile(aoi_vals, 25)),
                "p50": float(np.percentile(aoi_vals, 50)),
                "p75": float(np.percentile(aoi_vals, 75)),
                "max": float(np.max(aoi_vals)),
            },
        },
    }

    out_dir = os.path.dirname(args.output_json)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output_json, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)


if __name__ == "__main__":
    main()
