#!/usr/bin/env python3
import argparse
import concurrent.futures as cf
import datetime as dt
import os
import shlex
import statistics
import subprocess
from pathlib import Path
from tqdm import tqdm


def _to_num(v: str):
    try:
        if "." in v or "e" in v.lower():
            return float(v)
        return int(v)
    except ValueError:
        return v


def parse_kv_line(line: str):
    out = {}
    for tok in line.strip().split(","):
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        out[k] = _to_num(v)
    return out


def parse_summary_file(path: Path):
    agg = None
    ues = []
    if not path.exists():
        return agg, ues
    with path.open("r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("simTime="):
                agg = parse_kv_line(line)
            elif line.startswith("ue="):
                ues.append(parse_kv_line(line))
    return agg, ues


def run_once(args, out_dir: Path, run_idx: int, case_name: str = "default", overrides=None):
    overrides = overrides or {}
    summary_path = out_dir / f"summary_{case_name}_run{run_idx}.csv"
    random_run = int(overrides.get("randomRun", args.random_run))
    random_seed = int(overrides.get("randomSeed", args.random_seed))
    driver = str(overrides.get("driver", "ns3")).lower()
    cmd_args = [
        f"--numUes={overrides.get('numUes', args.num_ues)}",
        f"--simTime={overrides.get('simTime', args.sim_time)}",
        f"--appStart={overrides.get('appStart', args.app_start)}",
        f"--bwpBaseline={overrides.get('bwpBaseline', args.bwp_baseline)}",
        f"--bwpQueueThreshold={overrides.get('bwpQueueThreshold', args.bwp_queue_threshold)}",
        f"--envStepTime={overrides.get('envStepTime', args.env_step_time)}",
        f"--schedulerPolicy={overrides.get('schedulerPolicy', args.scheduler_policy)}",
        f"--ageOptimalGammaPenalty={overrides.get('ageOptimalGammaPenalty', args.age_optimal_gamma_penalty)}",
        f"--tpsDeadlineMs={overrides.get('tpsDeadlineMs', args.tps_deadline_ms)}",
        f"--dgsDelayTargetMs={overrides.get('dgsDelayTargetMs', args.dgs_delay_target_ms)}",
        f"--summaryFile={summary_path}",
        f"--dppV={overrides.get('dppV', args.dpp_v)}",
        f"--dppLambdaSwitch={overrides.get('dppLambdaSwitch', args.dpp_lambda_switch)}",
        f"--dppLambdaBler={overrides.get('dppLambdaBler', args.dpp_lambda_bler)}",
        f"--dppEpochMinIntervalS={overrides.get('dppEpochMinIntervalS', args.dpp_epoch_min_interval_s)}",
        f"--randomSeed={random_seed}",
        f"--randomRun={random_run}",
    ]
    if "initialBwpId" in overrides:
        cmd_args.append(f"--initialBwpId={int(overrides['initialBwpId'])}")
    if "enableMcsSwitch" in overrides:
        cmd_args.append(f"--enableMcsSwitch={1 if overrides['enableMcsSwitch'] else 0}")

    if args.extra:
        cmd_args.extend(args.extra)

    if driver == "drqn":
        model_path = str(overrides.get("drqnModelPath", args.drqn_model_path))
        python_bin = str(overrides.get("pythonBin", args.python_bin))
        open_gym_port = int(overrides.get("openGymPort", args.open_gym_port_base))
        ns3_arg_list = cmd_args.copy()
        full_cmd = [
            python_bin,
            "scratch/rl_bwp/eval_drqn.py",
            "--backend",
            "ns3",
            "--model-path",
            model_path,
            "--episodes",
            "1",
            "--seed",
            str(random_seed),
            "--num-ues",
            str(overrides.get("numUes", args.num_ues)),
            "--step-time-s",
            str(overrides.get("envStepTime", args.env_step_time)),
            "--episode-time-s",
            str(overrides.get("simTime", args.sim_time)),
            "--port",
            str(open_gym_port),
            "--start-sim",
            "--ns3-script",
            "aoi-prb-urban-appmix",
        ]
        for kv in ns3_arg_list:
            if kv.startswith("--"):
                full_cmd.extend(["--ns3-arg", kv[2:]])
    else:
        ns3_run_str = f"scratch/aoi-prb-urban-appmix {' '.join(cmd_args)}"
        full_cmd = ["./ns3", "run", ns3_run_str]

    print(f"\n[{case_name} run {run_idx}] {shlex.join(full_cmd)}")
    cp = subprocess.run(full_cmd, text=True, capture_output=True)
    if cp.returncode != 0:
        print(cp.stdout)
        print(cp.stderr)
        raise RuntimeError(f"ns3 run failed (run={run_idx}, code={cp.returncode})")

    agg, ues = parse_summary_file(summary_path)
    if not agg:
        raise RuntimeError(f"summary parse failed: {summary_path}")
    return agg, ues, summary_path


def fmt(v):
    if isinstance(v, float):
        return f"{v:.6f}"
    return str(v)


def print_run_stats(run_idx: int, agg: dict, ues: list):
    keys = [
        "runMeanThrMbps",
        "runMeanAoiMs",
        "runMeanPrbUtility",
        "runQueueOverflowRatio",
        "runMeanSwitchCount",
        "runTotalSwitchCount",
        "bler",
        "avgTbler",
        "losRatio",
        "nlosRatio",
        "thrLosMbps",
        "thrNlosMbps",
        "aoiLosMs",
        "aoiNlosMs",
    ]
    print(f"[run {run_idx}] aggregate")
    for k in keys:
        if k in agg:
            print(f"  {k:20s}: {fmt(agg[k])}")

    if ues:
        thr = [float(u.get("thrMbps", 0.0)) for u in ues]
        aoi = [float(u.get("aoiNodeMs", 0.0)) for u in ues]
        print(f"[run {run_idx}] UE stats")
        print(f"  ue_count              : {len(ues)}")
        print(f"  thrMbps mean/min/max  : {statistics.fmean(thr):.6f}/{min(thr):.6f}/{max(thr):.6f}")
        print(f"  aoiNodeMs mean/min/max: {statistics.fmean(aoi):.6f}/{min(aoi):.6f}/{max(aoi):.6f}")


def print_case_delta(name_a: str, agg_a: dict, name_b: str, agg_b: dict):
    print(f"\n[delta] {name_b} - {name_a}")
    for key in ("runMeanThrMbps", "runMeanAoiMs", "runMeanSwitchCount", "runTotalSwitchCount", "bler", "avgTbler"):
        if key not in agg_a or key not in agg_b:
            continue
        delta = float(agg_b[key]) - float(agg_a[key])
        print(f"  {key:20s}: {delta:+.6f}")


def print_multi_run_stats(all_agg: list):
    if len(all_agg) <= 1:
        return
    print("\n[multi-run] aggregate mean/stdev")
    metrics = [
        "runMeanThrMbps",
        "runMeanAoiMs",
        "runMeanPrbUtility",
        "runQueueOverflowRatio",
        "runMeanSwitchCount",
        "runTotalSwitchCount",
        "bler",
        "avgTbler",
    ]
    for m in metrics:
        vals = [float(a[m]) for a in all_agg if m in a]
        if not vals:
            continue
        mean_v = statistics.fmean(vals)
        std_v = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        print(f"  {m:20s}: {mean_v:.6f} / {std_v:.6f}")


def build_16_combo_cases(args):
    bwp_modes = ["BWP0ONLY", "BWP1ONLY", "threshold", "DRQN"]
    if args.include_dpp:
        bwp_modes.append("DPP")
    schedulers = ["rr", "aequitas", "tps", "dgs"]
    out = []
    for bwp in bwp_modes:
        for sched in schedulers:
            overrides = {
                "schedulerPolicy": sched,
                "enableMcsSwitch": False,
            }
            if bwp == "BWP0ONLY":
                overrides.update({"bwpBaseline": "none", "initialBwpId": 0})
            elif bwp == "BWP1ONLY":
                overrides.update({"bwpBaseline": "none", "initialBwpId": 1})
            elif bwp == "threshold":
                overrides.update({"bwpBaseline": "queue"})
            elif bwp == "DRQN":
                overrides.update(
                    {
                        "driver": "drqn",
                        "bwpBaseline": "none",
                        "drqnModelPath": args.drqn_model_path,
                        "pythonBin": args.python_bin,
                    }
                )
            elif bwp == "DPP":
                overrides.update({"bwpBaseline": "dpp"})
            case_name = f"{bwp.lower()}_{sched}"
            out.append((case_name, overrides))
    return out


def main():
    p = argparse.ArgumentParser(description="Runner for scratch/aoi-prb-urban-appmix with summary stats")
    p.add_argument("--runs", type=int, default=1)
    p.add_argument("--num-ues", type=int, default=20)
    p.add_argument("--sim-time", type=float, default=10.0)
    p.add_argument("--app-start", type=float, default=0.1)
    p.add_argument("--env-step-time", type=float, default=0.02)
    p.add_argument("--bwp-baseline", type=str, default="dpp")
    p.add_argument("--bwp-queue-threshold", type=int, default=800)
    p.add_argument("--scheduler-policy",
                   type=str,
                   default="rr",
                   choices=["rr", "pf", "aequitas", "age_optimal", "tps", "dgs"])
    p.add_argument("--age-optimal-gamma-penalty", type=float, default=20.0)
    p.add_argument("--tps-deadline-ms", type=float, default=100.0)
    p.add_argument("--dgs-delay-target-ms", type=float, default=100.0)
    p.add_argument("--dpp-v", type=float, default=0.5)
    p.add_argument("--dpp-lambda-switch", type=float, default=0.005)
    p.add_argument("--dpp-lambda-bler", type=float, default=1.0)
    p.add_argument("--dpp-epoch-min-interval-s", type=float, default=0.01)
    p.add_argument("--random-seed", type=int, default=1)
    p.add_argument("--random-run", type=int, default=1)
    p.add_argument("--compare-fixed-bwp-aequitas", action="store_true")
    p.add_argument("--run-16-combos", action="store_true")
    p.add_argument("--include-dpp",
                   action="store_true",
                   help="When used with --run-16-combos, add DPP x scheduler 4 cases (total 20 combos).")
    p.add_argument("--max-workers", type=int, default=4)
    p.add_argument("--drqn-model-path",
                   type=str,
                   default="scratch/rl_bwp/runs/drqn/final_model.pt")
    p.add_argument("--python-bin", type=str, default="ns3gym-venv/bin/python")
    p.add_argument("--open-gym-port-base", type=int, default=5600)
    p.add_argument("--out-dir", type=str, default="")
    p.add_argument("--extra", nargs=argparse.REMAINDER, default=[])
    args = p.parse_args()

    ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir) if args.out_dir else Path("scratch/rl_bwp/runs") / f"runner_{ts}"
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.run_16_combos:
        cases = build_16_combo_cases(args)
        case_aggs = {name: [] for name, _ in cases}
        futures = {}
        with cf.ThreadPoolExecutor(max_workers=max(1, args.max_workers)) as ex:
            for i in range(1, args.runs + 1):
                rr = args.random_run + (i - 1)
                for case_idx, (case_name, base_overrides) in enumerate(cases):
                    overrides = dict(base_overrides)
                    overrides["randomSeed"] = args.random_seed
                    overrides["randomRun"] = rr
                    if str(overrides.get("driver", "ns3")).lower() == "drqn":
                        overrides["openGymPort"] = (
                            args.open_gym_port_base + (i - 1) * len(cases) + case_idx
                        )
                    fut = ex.submit(run_once, args, out_dir, i, case_name, overrides)
                    futures[fut] = (i, case_name)
            
            total_jobs = len(cases) * args.runs
            pbar = tqdm(total=total_jobs, desc="Running")

            for fut in cf.as_completed(futures):
                i, case_name = futures[fut]
                agg, ues, summary_path = fut.result()
                print_run_stats(i, agg, ues)
                print(f"[{case_name} run {i}] summary: {summary_path}")
                case_aggs[case_name].append(agg)
                pbar.update(1)

        for case_name in sorted(case_aggs.keys()):
            print(f"\n[multi-run] {case_name}")
            print_multi_run_stats(case_aggs[case_name])
    elif args.compare_fixed_bwp_aequitas:
        case0_aggs = []
        case1_aggs = []
        for i in range(1, args.runs + 1):
            rr = args.random_run + (i - 1)
            common = {
                "bwpBaseline": "none",
                "schedulerPolicy": "aequitas",
                "enableMcsSwitch": False,
                "randomSeed": args.random_seed,
                "randomRun": rr,
            }
            agg0, ues0, p0 = run_once(args, out_dir, i, "fixed_bwp0_aequitas",
                                      {**common, "initialBwpId": 0})
            print_run_stats(i, agg0, ues0)
            print(f"[fixed_bwp0_aequitas run {i}] summary: {p0}")
            agg1, ues1, p1 = run_once(args, out_dir, i, "fixed_bwp1_aequitas",
                                      {**common, "initialBwpId": 1})
            print_run_stats(i, agg1, ues1)
            print(f"[fixed_bwp1_aequitas run {i}] summary: {p1}")
            print_case_delta("fixed_bwp0_aequitas", agg0, "fixed_bwp1_aequitas", agg1)
            case0_aggs.append(agg0)
            case1_aggs.append(agg1)

        print("\n[multi-run] fixed_bwp0_aequitas")
        print_multi_run_stats(case0_aggs)
        print("\n[multi-run] fixed_bwp1_aequitas")
        print_multi_run_stats(case1_aggs)
    else:
        all_agg = []
        for i in range(1, args.runs + 1):
            rr = args.random_run + (i - 1)
            agg, ues, summary_path = run_once(args, out_dir, i, "default", {"randomRun": rr})
            print_run_stats(i, agg, ues)
            print(f"[run {i}] summary: {summary_path}")
            all_agg.append(agg)

        print_multi_run_stats(all_agg)
    print(f"\noutput_dir={out_dir}")


if __name__ == "__main__":
    main()
