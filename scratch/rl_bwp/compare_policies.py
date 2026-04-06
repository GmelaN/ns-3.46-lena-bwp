#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shlex
import signal
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List


@dataclass
class RunResult:
    policy: str
    ok: bool
    metrics: Dict[str, float]
    note: str
    log_path: Path


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    default_python = root / "ns3gym-venv" / "bin" / "python"
    parser = argparse.ArgumentParser(
        description="Compare baseline policies (none/dt/dqn) against recurrent PPO and print KPI table."
    )
    parser.add_argument(
        "--policies",
        type=str,
        default="none,dt,dqn,rppo",
        help="Comma-separated policies from: none,dt,dqn,rppo",
    )
    parser.add_argument("--num-ues", type=int, default=10)
    parser.add_argument("--sim-time", type=float, default=10.0, help="Used for baseline runs.")
    parser.add_argument("--episode-time-s", type=float, default=10.0, help="Used for RPPO eval.")
    parser.add_argument("--episodes", type=int, default=3, help="Used for RPPO eval.")
    parser.add_argument("--env-step-time", type=float, default=0.02)
    parser.add_argument("--switch-delay-ms", type=float, default=10.0)
    parser.add_argument("--ns3-script", type=str, default="aoi-prb-urban-onoff")
    parser.add_argument("--model-path", type=str, default="scratch/rl_bwp/runs/10ues/final_model.zip")
    parser.add_argument(
        "--vecnorm-path",
        type=str,
        default="scratch/rl_bwp/runs/10ues/vecnormalize.pkl",
    )
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument(
        "--auto-match-rppo-num-ues",
        action="store_true",
        default=True,
        help="Auto override --num-ues for RPPO from model run_config.json when available.",
    )
    parser.add_argument(
        "--python-bin",
        type=str,
        default=str(default_python),
        help="Python interpreter for eval_rppo.py (must have sb3_contrib installed).",
    )
    parser.add_argument(
        "--extra-ns3-arg",
        action="append",
        default=[],
        help="Extra ns-3 arg key=value passed to RPPO eval. Repeatable.",
    )
    parser.add_argument(
        "--output-md",
        type=str,
        default="scratch/rl_bwp/policy_comparison.md",
    )
    parser.add_argument(
        "--output-csv",
        type=str,
        default="scratch/rl_bwp/policy_comparison.csv",
    )
    parser.add_argument(
        "--logs-dir",
        type=str,
        default="scratch/rl_bwp/compare_logs",
    )
    parser.add_argument("--timeout-sec", type=int, default=900)
    return parser.parse_args()


def run_cmd(cmd: List[str], cwd: Path, timeout: int) -> subprocess.CompletedProcess:
    proc = subprocess.Popen(
        cmd,
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    try:
        stdout, stderr = proc.communicate(timeout=timeout)
        return subprocess.CompletedProcess(args=cmd, returncode=proc.returncode, stdout=stdout, stderr=stderr)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            stdout, stderr = proc.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            stdout, stderr = proc.communicate()
        if stderr:
            stderr += "\n"
        stderr += f"[timeout] command exceeded {timeout}s"
        return subprocess.CompletedProcess(args=cmd, returncode=124, stdout=stdout, stderr=stderr)


def parse_baseline_metrics(text: str) -> Dict[str, float]:
    ue_re = re.compile(
        r"UE\d+\s+thr=([0-9.]+)\s+Mbps\s+AoI\(burst\)=([0-9.]+)\s+ms\s+AoI\(bg\)=([0-9.]+)\s+ms.*?bytes/PRB=([0-9.]+)"
    )
    agg_re = re.compile(r"=== Aggregate PRB .*?bytes/PRB=([0-9.]+)", re.DOTALL)
    thrs: List[float] = []
    delays: List[float] = []
    for m in ue_re.finditer(text):
        thr = float(m.group(1))
        aoi_b = float(m.group(2))
        aoi_bg = float(m.group(3))
        thrs.append(thr)
        delays.append(0.5 * (aoi_b + aoi_bg))

    out: Dict[str, float] = {}
    if thrs:
        out["mean_thr_mbps"] = sum(thrs) / len(thrs)
        out["sum_thr_mbps"] = sum(thrs)
        denom = len(thrs) * sum(x * x for x in thrs)
        out["jain_thr"] = (sum(thrs) ** 2 / denom) if denom > 0 else 0.0
    if delays:
        mean_delay = sum(delays) / len(delays)
        out["mean_delay_ms"] = mean_delay
        out["mean_aoi_ms"] = mean_delay
    agg = agg_re.search(text)
    if agg:
        out["bytes_per_prb"] = float(agg.group(1))
    return out


def parse_rppo_metrics(text: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    block_re = re.compile(
        r"KPI \(episode mean across steps\):\n(.*?)(?:\n\nKPI \(episode end value\):|\Z)",
        re.DOTALL,
    )
    line_re = re.compile(r"^([a-zA-Z0-9_]+):\s+mean=([-+0-9.eE]+),\s+std=([-+0-9.eE]+)\s*$")
    m = block_re.search(text)
    if not m:
        return out
    block = m.group(1).strip().splitlines()
    for line in block:
        lm = line_re.match(line.strip())
        if not lm:
            continue
        key = lm.group(1)
        out[key] = float(lm.group(2))
    return out


def write_markdown(path: Path, results: List[RunResult], args: argparse.Namespace) -> None:
    metrics_order = [
        "mean_thr_mbps",
        "sum_thr_mbps",
        "mean_delay_ms",
        "mean_aoi_ms",
        "mean_prb_utility",
        "queue_overflow_ratio",
        "switch_count",
        "jain_thr",
        "bytes_per_prb",
    ]
    lines = []
    lines.append("# Policy Comparison")
    lines.append("")
    lines.append(
        f"- num_ues={args.num_ues}, sim_time={args.sim_time}, episode_time_s={args.episode_time_s}, episodes={args.episodes}"
    )
    lines.append(f"- ns3_script={args.ns3_script}")
    lines.append("")
    header = ["policy", "status"] + metrics_order + ["note", "log"]
    lines.append("| " + " | ".join(header) + " |")
    lines.append("|" + "|".join(["---"] * len(header)) + "|")
    for r in results:
        row = [r.policy, "ok" if r.ok else "fail"]
        for k in metrics_order:
            if k in r.metrics:
                row.append(f"{r.metrics[k]:.4f}")
            else:
                row.append("NA")
        row.append(r.note)
        row.append(str(r.log_path))
        lines.append("| " + " | ".join(row) + " |")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_csv(path: Path, results: List[RunResult]) -> None:
    keys = set()
    for r in results:
        keys |= set(r.metrics.keys())
    metric_keys = sorted(keys)
    fieldnames = ["policy", "status", "note", "log_path"] + metric_keys
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in results:
            row = {
                "policy": r.policy,
                "status": "ok" if r.ok else "fail",
                "note": r.note,
                "log_path": str(r.log_path),
            }
            row.update(r.metrics)
            w.writerow(row)


def run_baseline(policy: str, args: argparse.Namespace, root: Path, log_path: Path) -> RunResult:
    run_arg = (
        f"scratch/{args.ns3_script} "
        f"--numUes={args.num_ues} "
        f"--simTime={args.sim_time} "
        f"--enableOpenGym=false "
        f"--bwpBaseline={policy} "
        f"--envStepTime={args.env_step_time} "
        f"--switchDelayMs={args.switch_delay_ms}"
    )
    cmd = ["./ns3", "run", run_arg]
    cp = run_cmd(cmd, root, timeout=args.timeout_sec)
    text = (cp.stdout or "") + ("\n" + cp.stderr if cp.stderr else "")
    log_path.write_text(text, encoding="utf-8")
    if cp.returncode != 0:
        return RunResult(policy=policy, ok=False, metrics={}, note=f"exit={cp.returncode}", log_path=log_path)
    metrics = parse_baseline_metrics(text)
    note = "baseline summary parsed" if metrics else "no baseline metrics parsed"
    return RunResult(policy=policy, ok=True, metrics=metrics, note=note, log_path=log_path)


def run_rppo(args: argparse.Namespace, root: Path, log_path: Path) -> RunResult:
    num_ues = args.num_ues
    run_cfg = Path(args.model_path).resolve().parent / "run_config.json"
    note_prefix = ""
    if args.auto_match_rppo_num_ues and run_cfg.exists():
        try:
            cfg = json.loads(run_cfg.read_text(encoding="utf-8"))
            cfg_num_ues = cfg.get("num_ues")
            if cfg_num_ues is None and isinstance(cfg.get("env"), dict):
                cfg_num_ues = cfg["env"].get("num_ues")
            if cfg_num_ues is None:
                cfg_num_ues = num_ues
            cfg_num_ues = int(cfg_num_ues)
            if cfg_num_ues != num_ues:
                note_prefix = f"num_ues auto-adjusted {num_ues}->{cfg_num_ues}; "
                num_ues = cfg_num_ues
        except Exception:
            pass

    cmd = [
        args.python_bin,
        "scratch/rl_bwp/eval_rppo.py",
        "--backend",
        "ns3",
        "--start-sim",
        "--ns3-script",
        args.ns3_script,
        "--model-path",
        args.model_path,
        "--vecnorm-path",
        args.vecnorm_path,
        "--num-ues",
        str(num_ues),
        "--episode-time-s",
        str(args.episode_time_s),
        "--episodes",
        str(args.episodes),
        "--switch-delay-ms",
        str(args.switch_delay_ms),
        "--port",
        str(args.port),
        "--step-time-s",
        str(args.env_step_time),
    ]
    for item in args.extra_ns3_arg:
        cmd.extend(["--ns3-arg", item])
    cp = run_cmd(cmd, root, timeout=args.timeout_sec)
    text = (cp.stdout or "") + ("\n" + cp.stderr if cp.stderr else "")
    cleaned = False
    if cp.returncode == 124:
        patt = f"{args.ns3_script} --openGymPort={args.port}"
        subprocess.run(["pkill", "-f", patt], cwd=str(root), check=False)
        cleaned = True
        text += f"\n[cleanup] pkill -f {patt}"
    log_path.write_text(text, encoding="utf-8")
    if cp.returncode != 0:
        return RunResult(
            policy="rppo",
            ok=False,
            metrics={},
            note=f"{note_prefix}exit={cp.returncode}" + ("; cleaned_orphan_ns3" if cleaned else ""),
            log_path=log_path,
        )
    metrics = parse_rppo_metrics(text)
    note = ("rppo KPI parsed" if metrics else "no rppo KPI parsed")
    note = note_prefix + note
    return RunResult(policy="rppo", ok=True, metrics=metrics, note=note, log_path=log_path)


def main() -> int:
    args = parse_args()
    root = Path(__file__).resolve().parents[2]
    logs_dir = (root / args.logs_dir).resolve()
    logs_dir.mkdir(parents=True, exist_ok=True)

    selected = [p.strip().lower() for p in args.policies.split(",") if p.strip()]
    allowed = {"none", "dt", "dqn", "rppo"}
    bad = [p for p in selected if p not in allowed]
    if bad:
        print(f"Unsupported policies: {bad}", file=sys.stderr)
        return 2

    results: List[RunResult] = []
    for p in selected:
        print(f"[run] policy={p}", flush=True)
        safe_name = re.sub(r"[^a-zA-Z0-9_.-]+", "_", p)
        log_path = logs_dir / f"{safe_name}.log"
        if p == "rppo":
            results.append(run_rppo(args, root, log_path))
        else:
            results.append(run_baseline(p, args, root, log_path))

    out_md = (root / args.output_md).resolve()
    out_csv = (root / args.output_csv).resolve()
    write_markdown(out_md, results, args)
    write_csv(out_csv, results)

    print(f"[done] markdown={out_md}")
    print(f"[done] csv={out_csv}")
    for r in results:
        print(f"[{r.policy}] {'ok' if r.ok else 'fail'} note={r.note} log={r.log_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
