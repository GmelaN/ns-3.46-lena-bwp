#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import re
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

SCHEDULER_PATTERN = r"(rr|pf|aequitas|age_optimal|tps|dgs)"
SUMMARY_RE = re.compile(rf"^summary_(.+)_{SCHEDULER_PATTERN}_run(\d+)\.csv$")


def parse_kv_line(line: str) -> Dict[str, float | str]:
    out: Dict[str, float | str] = {}
    for tok in line.strip().split(","):
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        k = k.strip()
        v = v.strip()
        try:
            out[k] = float(v)
        except ValueError:
            out[k] = v
    return out


def _pick_float(d: Dict[str, float | str], keys: List[str], default: float = float("nan")) -> float:
    for k in keys:
        if k in d and isinstance(d[k], (int, float)):
            return float(d[k])
    return default


def load_records(runner_dir: Path) -> List[Dict[str, float | str]]:
    records: List[Dict[str, float | str]] = []
    for path in sorted(runner_dir.glob("summary_*.csv")):
        m = SUMMARY_RE.match(path.name)
        if not m:
            continue
        mechanism = m.group(1)
        scheduler = m.group(2)
        run_idx = int(m.group(3))

        lines = path.read_text(encoding="utf-8").splitlines()
        if not lines:
            continue
        head = parse_kv_line(lines[0])

        rec: Dict[str, float | str] = {
            "mechanism": mechanism,
            "scheduler": scheduler,
            "run": run_idx,
            "summary_file": str(path),
            "goodput_mbps": _pick_float(head, ["dppServiceRateGoodputMbps", "runMeanThrMbps"]),
            "aoi_ms": _pick_float(head, ["packetAoiMeanMs", "runMeanAoiMs"]),
            "throughput_mbps": _pick_float(head, ["runMeanThrMbps"]),
            "bwp0_occupancy_ratio": _pick_float(head, ["bwp0OccupancyRatio"]),
            "bwp1_occupancy_ratio": _pick_float(head, ["bwp1OccupancyRatio"]),
            "total_switch_count": _pick_float(head, ["runTotalSwitchCount", "actualBwpSwitchExecTotal"]),
        }
        records.append(rec)
    return records


def _mean_std(values: List[float]) -> Tuple[float, float]:
    if not values:
        return (float("nan"), float("nan"))
    if len(values) == 1:
        return (values[0], 0.0)
    return (statistics.fmean(values), statistics.pstdev(values))


def aggregate(records: List[Dict[str, float | str]]) -> List[Dict[str, float | str]]:
    buckets: Dict[Tuple[str, str], List[Dict[str, float | str]]] = defaultdict(list)
    for r in records:
        buckets[(str(r["mechanism"]), str(r["scheduler"]))].append(r)

    out: List[Dict[str, float | str]] = []
    for (mechanism, scheduler), items in sorted(buckets.items()):
        goodputs = [float(x["goodput_mbps"]) for x in items]
        aois = [float(x["aoi_ms"]) for x in items]
        thrs = [float(x["throughput_mbps"]) for x in items]
        bwp0s = [float(x["bwp0_occupancy_ratio"]) for x in items]
        bwp1s = [float(x["bwp1_occupancy_ratio"]) for x in items]
        sws = [float(x["total_switch_count"]) for x in items]

        goodput_mean, goodput_std = _mean_std(goodputs)
        aoi_mean, aoi_std = _mean_std(aois)
        thr_mean, thr_std = _mean_std(thrs)
        bwp0_mean, bwp0_std = _mean_std(bwp0s)
        bwp1_mean, bwp1_std = _mean_std(bwp1s)
        sw_mean, sw_std = _mean_std(sws)

        out.append(
            {
                "mechanism": mechanism,
                "scheduler": scheduler,
                "runs": len(items),
                "goodput_mbps_mean": goodput_mean,
                "goodput_mbps_std": goodput_std,
                "aoi_ms_mean": aoi_mean,
                "aoi_ms_std": aoi_std,
                "throughput_mbps_mean": thr_mean,
                "throughput_mbps_std": thr_std,
                "bwp0_occupancy_ratio_mean": bwp0_mean,
                "bwp0_occupancy_ratio_std": bwp0_std,
                "bwp1_occupancy_ratio_mean": bwp1_mean,
                "bwp1_occupancy_ratio_std": bwp1_std,
                "total_switch_count_mean": sw_mean,
                "total_switch_count_std": sw_std,
            }
        )
    return out


def write_csv(path: Path, rows: List[Dict[str, float | str]], fieldnames: List[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def build_pivot(
    agg_rows: List[Dict[str, float | str]], value_key: str
) -> Tuple[List[str], List[Dict[str, float | str]]]:
    schedulers = sorted({str(r["scheduler"]) for r in agg_rows})
    fieldnames = ["mechanism", *schedulers]
    by_mech: Dict[str, Dict[str, float]] = defaultdict(dict)
    for r in agg_rows:
        by_mech[str(r["mechanism"])][str(r["scheduler"])] = float(r[value_key])

    rows: List[Dict[str, float | str]] = []
    for mech in sorted(by_mech):
        row: Dict[str, float | str] = {"mechanism": mech}
        for s in schedulers:
            row[s] = by_mech[mech].get(s, "")
        rows.append(row)
    return fieldnames, rows


def print_summary_table(agg_rows: List[Dict[str, float | str]]) -> None:
    print("[matrix]")
    print("mechanism   scheduler    runs   goodput(Mbps)   AoI(ms)   throughput(Mbps)   bwp0   bwp1")
    print("----------  -----------  ----  --------------  --------  -----------------  -----  -----")
    for r in agg_rows:
        print(
            f"{str(r['mechanism']):10s}  {str(r['scheduler']):11s}  {int(r['runs']):4d}  "
            f"{float(r['goodput_mbps_mean']):14.3f}  {float(r['aoi_ms_mean']):8.3f}  "
            f"{float(r['throughput_mbps_mean']):17.3f}  "
            f"{float(r['bwp0_occupancy_ratio_mean']):5.3f}  {float(r['bwp1_occupancy_ratio_mean']):5.3f}"
        )


def print_pivot_table(title: str, fieldnames: List[str], rows: List[Dict[str, float | str]]) -> None:
    print(f"\n[{title}]")
    print(" | ".join(fieldnames))
    print("-|-".join("-" * len(h) for h in fieldnames))
    for row in rows:
        cols = []
        for k in fieldnames:
            v = row.get(k, "")
            if isinstance(v, (int, float)):
                cols.append(f"{float(v):7.3f}")
            else:
                cols.append(f"{str(v):9s}")
        print(" | ".join(cols))


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Parse runner summary CSVs and aggregate KPI trends by BWP mechanism x scheduler."
    )
    ap.add_argument("--runner-dir", required=True, type=Path, help="Path like scratch/rl_bwp/runs/runner_YYYYMMDD_HHMMSS")
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output directory for parsed CSV files (used only when --write-csv is set)",
    )
    ap.add_argument("--prefix", type=str, default="matrix", help="Output filename prefix")
    ap.add_argument("--write-csv", action="store_true", help="Also write parsed CSV outputs to disk")
    args = ap.parse_args()

    runner_dir = args.runner_dir
    if not runner_dir.exists():
        raise FileNotFoundError(f"runner-dir not found: {runner_dir}")

    records = load_records(runner_dir)
    if not records:
        raise RuntimeError(f"No summary_*.csv parsed under {runner_dir}")

    agg_rows = aggregate(records)

    raw_fields = [
        "mechanism",
        "scheduler",
        "run",
        "goodput_mbps",
        "aoi_ms",
        "throughput_mbps",
        "bwp0_occupancy_ratio",
        "bwp1_occupancy_ratio",
        "total_switch_count",
        "summary_file",
    ]
    agg_fields = [
        "mechanism",
        "scheduler",
        "runs",
        "goodput_mbps_mean",
        "goodput_mbps_std",
        "aoi_ms_mean",
        "aoi_ms_std",
        "throughput_mbps_mean",
        "throughput_mbps_std",
        "bwp0_occupancy_ratio_mean",
        "bwp0_occupancy_ratio_std",
        "bwp1_occupancy_ratio_mean",
        "bwp1_occupancy_ratio_std",
        "total_switch_count_mean",
        "total_switch_count_std",
    ]

    print_summary_table(agg_rows)

    piv_g_fields, piv_g_rows = build_pivot(agg_rows, "goodput_mbps_mean")
    piv_a_fields, piv_a_rows = build_pivot(agg_rows, "aoi_ms_mean")

    print_pivot_table("pivot_goodput", piv_g_fields, piv_g_rows)
    print_pivot_table("pivot_aoi", piv_a_fields, piv_a_rows)

    print(f"\nparsed_records={len(records)}")

    if args.write_csv:
        out_dir = args.out_dir if args.out_dir is not None else (runner_dir / "parsed")
        out_dir.mkdir(parents=True, exist_ok=True)
        raw_path = out_dir / f"{args.prefix}_raw.csv"
        agg_path = out_dir / f"{args.prefix}_agg.csv"
        piv_g_path = out_dir / f"{args.prefix}_pivot_goodput.csv"
        piv_a_path = out_dir / f"{args.prefix}_pivot_aoi.csv"
        write_csv(raw_path, records, raw_fields)
        write_csv(agg_path, agg_rows, agg_fields)
        write_csv(piv_g_path, piv_g_rows, piv_g_fields)
        write_csv(piv_a_path, piv_a_rows, piv_a_fields)
        print(f"raw_csv={raw_path}")
        print(f"agg_csv={agg_path}")
        print(f"pivot_goodput_csv={piv_g_path}")
        print(f"pivot_aoi_csv={piv_a_path}")


if __name__ == "__main__":
    main()
