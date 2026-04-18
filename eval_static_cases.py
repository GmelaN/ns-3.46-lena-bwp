#!/usr/bin/env python3
import subprocess
import os
import csv
import re
from concurrent.futures import ProcessPoolExecutor

def run_simulation(case_info):
    label, bwp_id, seed = case_info
    print(f"--- Starting {label}: BWP={bwp_id} ---")
    summary_file = f"summary_{label}.csv"
    if os.path.exists(summary_file):
        os.remove(summary_file)
    
    # Simulation arguments for the script itself
    sim_args = (
        f"aoi-prb-urban-appmix --numUes=20 --simTime=10 --initialBwpId={bwp_id} "
        f"--enableOpenGym=0 --summaryFile={summary_file}"
    )
    
    # ns-3 runtime arguments must come after '--'
    cmd = [
        "./ns3", "run", sim_args, "--", f"--RngRun={seed}"
    ]
    
    try:
        # We don't use shell=True, so cmd is a list
        subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(f"--- Finished {label} ---")
    except subprocess.CalledProcessError as e:
        print(f"Error running simulation for {label}:\n{e.stderr}")
        return None
    
    return label, summary_file

def parse_summary(file_path, case_label):
    results = []
    if not os.path.exists(file_path):
        return results
    
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith("ue="):
                try:
                    data = dict(item.split("=") for item in line.strip().split(","))
                    data['case'] = case_label
                    results.append(data)
                except ValueError:
                    continue
    return results

def main():
    seed = 1
    cases = [
        ("BWP0",  0, seed),
        ("BWP1",  1, seed),
    ]
    
    all_ue_results = []
    output_csv = "static_cases_per_ue_results.csv"
    
    print(f"Launching {len(cases)} simulations in parallel...")
    with ProcessPoolExecutor() as executor:
        finished_cases = list(executor.map(run_simulation, cases))
    
    for res in finished_cases:
        if res:
            label, summary_path = res
            case_results = parse_summary(summary_path, label)
            all_ue_results.extend(case_results)
            if os.path.exists(summary_path):
                os.remove(summary_path)
    
    if all_ue_results:
        first_row = all_ue_results[0]
        keys = ['case'] + [k for k in first_row.keys() if k != 'case']
        
        with open(output_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(all_ue_results)
        print(f"\n--- All Simulations Complete! Combined results saved to: {output_csv} ---")
    else:
        print("No results collected. Check logs for errors.")

if __name__ == "__main__":
    main()
