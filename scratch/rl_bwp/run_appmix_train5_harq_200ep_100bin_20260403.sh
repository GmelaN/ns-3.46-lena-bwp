#!/usr/bin/env bash
set -euo pipefail

ROOT_REL="${1:-appmix_train5_ue20_sim2_step001_ep200_harq_100bin_$(date +%Y%m%d_%H%M%S)}"
RUN_ROOT="scratch/rl_bwp/runs/${ROOT_REL}"
LOG_DIR="${RUN_ROOT}/logs"
STATUS_CSV="${RUN_ROOT}/status.csv"

NS3_SCRIPT="aoi-prb-urban-appmix"
NUM_UES=20
TRAIN_SIM=2
STEP=0.01
TRAIN_EPISODES=200
TOTAL_ENV_STEPS=$(( NUM_UES * 200 * TRAIN_EPISODES ))
SEED_BASE=41000
REWARD_BIN=100
LOSS_BIN=100
SAVE_EVERY=50

mkdir -p "${RUN_ROOT}" "${LOG_DIR}"
echo "model,phase,train_pid,monitor_pid,status,start_ts,end_ts" > "${STATUS_CSV}"

if ! ./ns3 show profile | grep -q "Build profile: optimized"; then
  ./ns3 configure -d optimized
fi
./ns3 build scratch/aoi-prb-urban-appmix

prepare_monitor_script() {
  local job_root="$1"
  local tag="$2"
  local out_script="${job_root}/monitor_ns3_reconnector_${tag}.sh"
  cp scratch/rl_bwp/monitor_ns3_reconnector.sh "${out_script}"
  chmod +x "${out_script}"
  sed -i "s|LOG_DIR=\"\$ROOT_DIR/scratch/rl_bwp/runs/monitor_logs\"|LOG_DIR=\"\$ROOT_DIR/${job_root}/monitor_logs\"|g" "${out_script}"
  echo "${out_script}"
}

launch_train_job() {
  local phase="$1"
  local model_key="$2"
  local trainer="$3"      # rqrdqn | drqn
  local scheduler="$4"    # pf | aequitas
  local port="$5"
  local fixed_bwp="${6:-}"  # "" | 0 | 1

  local run_name="${ROOT_REL}/${model_key}_train"
  local job_root="scratch/rl_bwp/runs/${run_name}"
  local train_log="${LOG_DIR}/${model_key}_train.log"
  local monitor_log="${LOG_DIR}/${model_key}_monitor.log"
  local pid_file="${job_root}/train.pid"
  local mon_pid_file="${job_root}/monitor.pid"
  local start_ts
  start_ts="$(date '+%F %T%z')"

  mkdir -p "${job_root}"
  local mon_script
  mon_script="$(prepare_monitor_script "${job_root}" "${model_key}")"

  local -a train_cmd
  if [[ "${trainer}" == "rqrdqn" ]]; then
    train_cmd=(
      ./ns3gym-venv/bin/python3 -u scratch/rl_bwp/train_rqrdqn.py
      --backend ns3
      --run-name "${run_name}"
      --start-sim
      --ns3-script "${NS3_SCRIPT}"
      --num-ues "${NUM_UES}"
      --episode-time-s "${TRAIN_SIM}"
      --step-time-s "${STEP}"
      --seed "$((SEED_BASE + port))"
      --min-completed-episodes "${TRAIN_EPISODES}"
      --total-env-steps "${TOTAL_ENV_STEPS}"
      --reward-bin-size "${REWARD_BIN}"
      --loss-bin-size "${LOSS_BIN}"
      --save-every-episodes "${SAVE_EVERY}"
      --disable-step-reward-log
      --port "${port}"
      --ns3-arg trafficModel=mixed
      --ns3-arg schedulerPolicy="${scheduler}"
      --ns3-arg enableHarqReTx=true
    )
  else
    train_cmd=(
      ./ns3gym-venv/bin/python3 -u scratch/rl_bwp/train_drqn.py
      --backend ns3
      --run-name "${run_name}"
      --start-sim
      --ns3-script "${NS3_SCRIPT}"
      --num-ues "${NUM_UES}"
      --episode-time-s "${TRAIN_SIM}"
      --step-time-s "${STEP}"
      --seed "$((SEED_BASE + port))"
      --min-completed-episodes "${TRAIN_EPISODES}"
      --total-env-steps "${TOTAL_ENV_STEPS}"
      --reward-bin-size "${REWARD_BIN}"
      --loss-bin-size "${LOSS_BIN}"
      --save-every-episodes "${SAVE_EVERY}"
      --disable-step-reward-log
      --port "${port}"
      --ns3-arg trafficModel=mixed
      --ns3-arg schedulerPolicy="${scheduler}"
      --ns3-arg enableHarqReTx=true
    )
  fi

  if [[ "${scheduler}" == "aequitas" ]]; then
    train_cmd+=(--ns3-arg aequitasEnableMcsSelection=true)
  fi
  if [[ -n "${fixed_bwp}" ]]; then
    train_cmd+=(--ns3-arg enableRlBwpControl=false --ns3-arg initialBwpId="${fixed_bwp}")
  fi

  "${train_cmd[@]}" > "${train_log}" 2>&1 &
  local train_pid=$!
  echo "${train_pid}" > "${pid_file}"

  local -a mon_args=(
    "${mon_script}"
    "${train_pid}"
    "${port}"
    10
    5
    "${NS3_SCRIPT}"
    --numUes="${NUM_UES}"
    --simTime="${TRAIN_SIM}.0"
    --envStepTime="${STEP}"
    --enableOpenGym=true
    --trafficModel=mixed
    --schedulerPolicy="${scheduler}"
    --enableHarqReTx=true
  )
  if [[ "${scheduler}" == "aequitas" ]]; then
    mon_args+=(--aequitasEnableMcsSelection=true)
  fi
  if [[ "${trainer}" == "drqn" ]]; then
    mon_args+=(--rlDrqnProfile=true --enableRlMcsControl=false)
  else
    mon_args+=(--enableRlMcsControl=true)
  fi
  if [[ -n "${fixed_bwp}" ]]; then
    mon_args+=(--enableRlBwpControl=false --initialBwpId="${fixed_bwp}")
  fi

  nohup "${mon_args[@]}" > "${monitor_log}" 2>&1 &
  local mon_pid=$!
  echo "${mon_pid}" > "${mon_pid_file}"

  echo "${model_key},${phase},${train_pid},${mon_pid},running,${start_ts}," >> "${STATUS_CSV}"
}

finalize_job() {
  local model_key="$1"
  local phase="$2"
  local train_pid="$3"
  local mon_pid="$4"
  local status="$5"
  local end_ts
  end_ts="$(date '+%F %T%z')"
  kill "${mon_pid}" 2>/dev/null || true
  wait "${mon_pid}" 2>/dev/null || true
  echo "${model_key},${phase},${train_pid},${mon_pid},${status},,${end_ts}" >> "${STATUS_CSV}"
}

wait_phase_jobs() {
  local phase="$1"
  shift
  local -a jobs=("$@")
  local rc=0
  for job in "${jobs[@]}"; do
    IFS=":" read -r model_key train_pid mon_pid <<< "${job}"
    local st=0
    wait "${train_pid}" || st=$?
    if [[ "${st}" -ne 0 ]]; then
      rc=1
      finalize_job "${model_key}" "${phase}" "${train_pid}" "${mon_pid}" "failed(${st})"
    else
      finalize_job "${model_key}" "${phase}" "${train_pid}" "${mon_pid}" "done"
    fi
  done
  return "${rc}"
}

phase1_jobs=()
launch_train_job phase1 rqr_dqn_pf rqrdqn pf 68111
phase1_jobs+=("rqr_dqn_pf:$(cat "scratch/rl_bwp/runs/${ROOT_REL}/rqr_dqn_pf_train/train.pid"):$(cat "scratch/rl_bwp/runs/${ROOT_REL}/rqr_dqn_pf_train/monitor.pid")")
launch_train_job phase1 drqn_aequitas drqn aequitas 68211
phase1_jobs+=("drqn_aequitas:$(cat "scratch/rl_bwp/runs/${ROOT_REL}/drqn_aequitas_train/train.pid"):$(cat "scratch/rl_bwp/runs/${ROOT_REL}/drqn_aequitas_train/monitor.pid")")

if ! wait_phase_jobs phase1 "${phase1_jobs[@]}"; then
  echo "phase1 failed. check ${STATUS_CSV} and ${LOG_DIR}" >&2
  exit 1
fi

phase2_jobs=()
launch_train_job phase2 drqn_pf drqn pf 68311
phase2_jobs+=("drqn_pf:$(cat "scratch/rl_bwp/runs/${ROOT_REL}/drqn_pf_train/train.pid"):$(cat "scratch/rl_bwp/runs/${ROOT_REL}/drqn_pf_train/monitor.pid")")
launch_train_job phase2 bwp0_aequitas drqn aequitas 68411 0
phase2_jobs+=("bwp0_aequitas:$(cat "scratch/rl_bwp/runs/${ROOT_REL}/bwp0_aequitas_train/train.pid"):$(cat "scratch/rl_bwp/runs/${ROOT_REL}/bwp0_aequitas_train/monitor.pid")")
launch_train_job phase2 bwp1_aequitas drqn aequitas 68511 1
phase2_jobs+=("bwp1_aequitas:$(cat "scratch/rl_bwp/runs/${ROOT_REL}/bwp1_aequitas_train/train.pid"):$(cat "scratch/rl_bwp/runs/${ROOT_REL}/bwp1_aequitas_train/monitor.pid")")

if ! wait_phase_jobs phase2 "${phase2_jobs[@]}"; then
  echo "phase2 failed. check ${STATUS_CSV} and ${LOG_DIR}" >&2
  exit 1
fi

echo "all training jobs completed: ${RUN_ROOT}"
