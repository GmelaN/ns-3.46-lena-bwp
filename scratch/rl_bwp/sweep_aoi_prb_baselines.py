#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import re
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List


@dataclass
class RunRecord:
    baseline: str
    num_ues: int
    load_scale: float
    burst_rate_mbps: float
    background_rate_kbps: float
    offered_load_mbps: float
    metrics: Dict[str, float]
    summary_path: Path
    trace_path: Path
    log_path: Path


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="Sweep aoi-prb-urban-onoff baselines over UE counts and traffic load, then analyze PRB utility correlations."
    )
    parser.add_argument("--ns3-script", type=str, default="aoi-prb-urban-onoff")
    parser.add_argument("--baselines", type=str, default="none,dt")
    parser.add_argument("--num-ues", type=str, default="4,8,12")
    parser.add_argument("--load-scales", type=str, default="0.25,0.5,1.0,1.5,2.0")
    parser.add_argument("--sim-time", type=float, default=2.5)
    parser.add_argument("--app-start", type=float, default=0.3)
    parser.add_argument("--env-step-time", type=float, default=0.02)
    parser.add_argument("--switch-delay-ms", type=float, default=0.1)
    parser.add_argument("--burst-rate-mbps", type=float, default=100.0)
    parser.add_argument("--background-rate-kbps", type=float, default=500.0)
    parser.add_argument("--burst-on-ms", type=float, default=100.0)
    parser.add_argument("--burst-off-ms", type=float, default=150.0)
    parser.add_argument("--timeout-sec", type=int, default=900)
    parser.add_argument("--build", action="store_true", default=False)
    parser.add_argument("--analyze-existing", action="store_true", default=False)
    parser.add_argument(
        "--output-dir",
        type=str,
        default="scratch/rl_bwp/runs/sweep_aoi_prb_baselines",
    )
    return parser.parse_args()


def parse_csv_list(text: str, cast) -> List:
    return [cast(item.strip()) for item in text.split(",") if item.strip()]


def run_cmd(cmd: List[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd),
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )


def parse_keyval_line(line: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for token in line.strip().split(","):
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        key = key.strip()
        value = value.strip()
        try:
            out[key] = float(value)
        except ValueError:
            continue
    return out


def parse_summary_file(path: Path) -> Dict[str, float]:
    lines = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not lines:
        raise RuntimeError(f"empty summary file: {path}")
    for line in reversed(lines):
        if "runMeanThrMbps=" in line or ("simTime=" in line and "ue=" not in line):
            return parse_keyval_line(line)
    raise RuntimeError(f"aggregate summary line not found: {path}")


def pearson(xs: Iterable[float], ys: Iterable[float]) -> float:
    x = list(xs)
    y = list(ys)
    n = len(x)
    if n < 2:
        return math.nan
    mean_x = sum(x) / n
    mean_y = sum(y) / n
    cov = sum((a - mean_x) * (b - mean_y) for a, b in zip(x, y))
    var_x = sum((a - mean_x) ** 2 for a in x)
    var_y = sum((b - mean_y) ** 2 for b in y)
    if var_x <= 0.0 or var_y <= 0.0:
        return math.nan
    return cov / math.sqrt(var_x * var_y)


def fmt(value: float) -> str:
    if isinstance(value, float) and math.isnan(value):
        return "NA"
    return f"{value:.4f}"


def build_if_needed(root: Path, args: argparse.Namespace) -> None:
    if not args.build:
        return
    cp = run_cmd(["./ns3", "build"], root, args.timeout_sec)
    if cp.returncode != 0:
        sys.stderr.write(cp.stdout)
        sys.stderr.write(cp.stderr)
        raise RuntimeError("ns3 build failed")


def baseline_run_name(baseline: str, num_ues: int, load_scale: float) -> str:
    safe_scale = re.sub(r"[^0-9A-Za-z_.-]+", "_", f"{load_scale:.2f}")
    return f"{baseline}_ue{num_ues}_load{safe_scale}"


def run_sweep(root: Path, args: argparse.Namespace, out_dir: Path) -> List[RunRecord]:
    baselines = parse_csv_list(args.baselines, str)
    num_ues_values = parse_csv_list(args.num_ues, int)
    load_scales = parse_csv_list(args.load_scales, float)
    records: List[RunRecord] = []

    burst_duty = args.burst_on_ms / max(1e-9, args.burst_on_ms + args.burst_off_ms)
    for baseline in baselines:
        for num_ues in num_ues_values:
            for load_scale in load_scales:
                burst_rate_mbps = args.burst_rate_mbps * load_scale
                background_rate_kbps = args.background_rate_kbps * load_scale
                offered_per_ue_mbps = burst_rate_mbps * burst_duty + background_rate_kbps / 1000.0
                offered_load_mbps = offered_per_ue_mbps * num_ues

                run_name = baseline_run_name(baseline, num_ues, load_scale)
                summary_path = out_dir / "summaries" / f"{run_name}.txt"
                trace_path = out_dir / "traces" / f"{run_name}.csv"
                log_path = out_dir / "logs" / f"{run_name}.log"
                summary_path.parent.mkdir(parents=True, exist_ok=True)
                trace_path.parent.mkdir(parents=True, exist_ok=True)
                log_path.parent.mkdir(parents=True, exist_ok=True)

                run_arg = (
                    f"scratch/{args.ns3_script} "
                    f"--enableOpenGym=false "
                    f"--bwpBaseline={baseline} "
                    f"--numUes={num_ues} "
                    f"--simTime={args.sim_time} "
                    f"--appStart={args.app_start} "
                    f"--envStepTime={args.env_step_time} "
                    f"--switchDelayMs={args.switch_delay_ms} "
                    f"--burstRateMbps={burst_rate_mbps} "
                    f"--backgroundRateKbps={background_rate_kbps} "
                    f"--burstOnMs={args.burst_on_ms} "
                    f"--burstOffMs={args.burst_off_ms} "
                    f"--summaryFile={summary_path} "
                    f"--metricsTraceFile={trace_path}"
                )
                cmd = ["./ns3", "run", "--no-build", run_arg]
                print(f"[run] {baseline} numUes={num_ues} loadScale={load_scale:.2f}", flush=True)
                cp = run_cmd(cmd, root, args.timeout_sec)
                text = (cp.stdout or "") + ("\n" + cp.stderr if cp.stderr else "")
                log_path.write_text(text, encoding="utf-8")
                if cp.returncode != 0:
                    raise RuntimeError(
                        f"run failed for {run_name} (exit={cp.returncode}). See {log_path}"
                    )
                metrics = parse_summary_file(summary_path)
                records.append(
                    RunRecord(
                        baseline=baseline,
                        num_ues=num_ues,
                        load_scale=load_scale,
                        burst_rate_mbps=burst_rate_mbps,
                        background_rate_kbps=background_rate_kbps,
                        offered_load_mbps=offered_load_mbps,
                        metrics=metrics,
                        summary_path=summary_path,
                        trace_path=trace_path,
                        log_path=log_path,
                    )
                )
    return records


def load_existing_records(out_dir: Path) -> List[RunRecord]:
    records: List[RunRecord] = []
    summary_dir = out_dir / "summaries"
    for summary_path in sorted(summary_dir.glob("*.txt")):
        match = re.fullmatch(r"([a-z0-9_-]+)_ue(\d+)_load([0-9.]+)\.txt", summary_path.name)
        if not match:
            continue
        baseline = match.group(1)
        num_ues = int(match.group(2))
        load_scale = float(match.group(3))
        metrics = parse_summary_file(summary_path)
        burst_rate_mbps = float(metrics.get("burstRateMbps", math.nan))
        background_rate_kbps = float(metrics.get("backgroundRateKbps", math.nan))
        trace_path = out_dir / "traces" / summary_path.name.replace(".txt", ".csv")
        log_path = out_dir / "logs" / summary_path.name.replace(".txt", ".log")
        burst_duty = burst_rate_mbps / burst_rate_mbps if burst_rate_mbps > 0 else 1.0
        offered_per_ue_mbps = math.nan
        if not math.isnan(burst_rate_mbps) and not math.isnan(background_rate_kbps):
            burst_on_ms = 100.0
            burst_off_ms = 150.0
            burst_duty = burst_on_ms / (burst_on_ms + burst_off_ms)
            offered_per_ue_mbps = burst_rate_mbps * burst_duty + background_rate_kbps / 1000.0
        records.append(
            RunRecord(
                baseline=baseline,
                num_ues=num_ues,
                load_scale=load_scale,
                burst_rate_mbps=burst_rate_mbps,
                background_rate_kbps=background_rate_kbps,
                offered_load_mbps=offered_per_ue_mbps * num_ues,
                metrics=metrics,
                summary_path=summary_path,
                trace_path=trace_path,
                log_path=log_path,
            )
        )
    return records


def write_runs_csv(path: Path, records: List[RunRecord]) -> None:
    metric_keys = sorted({key for record in records for key in record.metrics.keys()})
    fieldnames = [
        "baseline",
        "num_ues",
        "load_scale",
        "burst_rate_mbps",
        "background_rate_kbps",
        "offered_load_mbps",
        "summary_path",
        "trace_path",
        "log_path",
    ] + metric_keys
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            row = {
                "baseline": record.baseline,
                "num_ues": record.num_ues,
                "load_scale": record.load_scale,
                "burst_rate_mbps": record.burst_rate_mbps,
                "background_rate_kbps": record.background_rate_kbps,
                "offered_load_mbps": record.offered_load_mbps,
                "summary_path": str(record.summary_path),
                "trace_path": str(record.trace_path),
                "log_path": str(record.log_path),
            }
            row.update(record.metrics)
            writer.writerow(row)


def summarize_correlations(records: List[RunRecord]) -> List[Dict[str, float]]:
    output: List[Dict[str, float]] = []
    baselines = sorted({record.baseline for record in records})
    for baseline in baselines:
        subset = [record for record in records if record.baseline == baseline]
        prb = [record.metrics.get("runMeanPrbUtility", math.nan) for record in subset]
        thr = [record.metrics.get("runMeanThrMbps", math.nan) for record in subset]
        aoi = [record.metrics.get("runMeanAoiMs", math.nan) for record in subset]
        bytes_per_prb = [record.metrics.get("bytesPerPrb", math.nan) for record in subset]
        output.append(
            {
                "baseline": baseline,
                "corr_prb_thr": pearson(prb, thr),
                "corr_prb_aoi": pearson(prb, aoi),
                "corr_prb_bytes_per_prb": pearson(prb, bytes_per_prb),
                "min_thr_mbps": min(thr),
                "max_thr_mbps": max(thr),
                "min_aoi_ms": min(aoi),
                "max_aoi_ms": max(aoi),
            }
        )
    return output


def write_correlations_csv(path: Path, rows: List[Dict[str, float]]) -> None:
    fieldnames = [
        "baseline",
        "corr_prb_thr",
        "corr_prb_aoi",
        "corr_prb_bytes_per_prb",
        "min_thr_mbps",
        "max_thr_mbps",
        "min_aoi_ms",
        "max_aoi_ms",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_markdown(
    path: Path,
    args: argparse.Namespace,
    records: List[RunRecord],
    corr_rows: List[Dict[str, float]],
) -> None:
    baselines = sorted({record.baseline for record in records})
    lines: List[str] = []
    lines.append("# AoI/PRB Baseline Sweep")
    lines.append("")
    lines.append(f"- baselines={args.baselines}")
    lines.append(f"- num_ues={args.num_ues}")
    lines.append(f"- load_scales={args.load_scales}")
    lines.append(f"- sim_time={args.sim_time}, app_start={args.app_start}, env_step_time={args.env_step_time}")
    lines.append(
        f"- base_burst_rate_mbps={args.burst_rate_mbps}, base_background_rate_kbps={args.background_rate_kbps}"
    )
    lines.append("")
    lines.append("## Correlations")
    lines.append("")
    lines.append(
        "| baseline | corr(PRB utility, throughput) | corr(PRB utility, AoI) | corr(PRB utility, bytes/PRB) |"
    )
    lines.append("|---|---:|---:|---:|")
    for row in corr_rows:
        lines.append(
            "| {baseline} | {corr_prb_thr} | {corr_prb_aoi} | {corr_prb_bytes_per_prb} |".format(
                baseline=row["baseline"],
                corr_prb_thr=fmt(row["corr_prb_thr"]),
                corr_prb_aoi=fmt(row["corr_prb_aoi"]),
                corr_prb_bytes_per_prb=fmt(row["corr_prb_bytes_per_prb"]),
            )
        )
    for baseline in baselines:
        lines.append("")
        lines.append(f"## Baseline `{baseline}`")
        lines.append("")
        lines.append(
            "| num_ues | load_scale | offered_load_mbps | runMeanThrMbps | runMeanAoiMs | runMeanPrbUtility | bytesPerPrb | runMeanSwitchCount |"
        )
        lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|")
        subset = [record for record in records if record.baseline == baseline]
        subset.sort(key=lambda r: (r.num_ues, r.load_scale))
        for record in subset:
            lines.append(
                "| {num_ues} | {load_scale} | {offered_load} | {thr} | {aoi} | {prb} | {bpp} | {switches} |".format(
                    num_ues=record.num_ues,
                    load_scale=fmt(record.load_scale),
                    offered_load=fmt(record.offered_load_mbps),
                    thr=fmt(record.metrics.get("runMeanThrMbps", math.nan)),
                    aoi=fmt(record.metrics.get("runMeanAoiMs", math.nan)),
                    prb=fmt(record.metrics.get("runMeanPrbUtility", math.nan)),
                    bpp=fmt(record.metrics.get("bytesPerPrb", math.nan)),
                    switches=fmt(record.metrics.get("runMeanSwitchCount", math.nan)),
                )
            )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    root = Path(__file__).resolve().parents[2]
    out_dir = (root / args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.analyze_existing:
        records = load_existing_records(out_dir)
    else:
        build_if_needed(root, args)
        records = run_sweep(root, args, out_dir)
    corr_rows = summarize_correlations(records)

    runs_csv = out_dir / "baseline_sweep_runs.csv"
    corr_csv = out_dir / "baseline_sweep_correlations.csv"
    report_md = out_dir / "baseline_sweep_report.md"
    write_runs_csv(runs_csv, records)
    write_correlations_csv(corr_csv, corr_rows)
    write_markdown(report_md, args, records, corr_rows)

    print(f"[done] runs_csv={runs_csv}")
    print(f"[done] correlations_csv={corr_csv}")
    print(f"[done] report_md={report_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
