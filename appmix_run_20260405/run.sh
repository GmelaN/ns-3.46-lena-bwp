#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RUN_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="${RUN_DIR}/logs"
BASELINE_DIR="${RUN_DIR}/baselines"
mkdir -p "$LOG_DIR" "$BASELINE_DIR"

NUM_UES=20
SIM_TIME=5
STEP=0.01
EPISODES=10
TOTAL_ENV_STEPS=$(( NUM_UES * 200 * EPISODES ))
NS3_SCRIPT="aoi-prb-urban-appmix"
PYTHON="${ROOT_DIR}/ns3gym-venv/bin/python3"
PORT_RQR=66100
PORT_DRQN_AEQ=66200
PORT_DRQN_PF=66300

ensure_optimized() {
  if ! "${ROOT_DIR}/ns3" show profile | grep -q "Build profile: optimized"; then
    "${ROOT_DIR}/ns3" configure -d optimized
  fi
  "${ROOT_DIR}/ns3" build "${NS3_SCRIPT}"
}

cleanup_leftovers() {
  pkill -f 'aoi-prb-urban-appmix' >/dev/null 2>&1 || true
  pkill -f 'monitor_ns3_reconnector.sh' >/dev/null 2>&1 || true
  for _ in $(seq 1 10); do
    if ! pgrep -af 'aoi-prb-urban-appmix' >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
}

pick_free_ports() {
  local base=$((62000 + (RANDOM % 2000)))
  PORT_RQR="${base}"
  PORT_DRQN_AEQ="$((base + 10))"
  PORT_DRQN_PF="$((base + 20))"
}

launch_rqrdqn_pf() {
  local log="${LOG_DIR}/rqrdqn_pf.log"
  echo "Launch RQR-DQN+PF -> ${log}"
  (
    "${PYTHON}" "${ROOT_DIR}/scratch/rl_bwp/train_rqrdqn.py" \
      --backend ns3 \
      --run-name "appmix_rqrdqn_pf_ue20_sim5_step010_ep10_harq" \
      --seed 1001 \
      --num-ues "${NUM_UES}" \
      --episode-time-s "${SIM_TIME}" \
      --step-time-s "${STEP}" \
      --min-completed-episodes "${EPISODES}" \
      --total-env-steps "${TOTAL_ENV_STEPS}" \
      --reward-bin-size 200 \
      --save-every-episodes 1 \
      --start-sim \
      --port "${PORT_RQR}" \
      --ns3-script "${NS3_SCRIPT}" \
      --ns3-arg trafficModel=mixed \
      --ns3-arg schedulerPolicy=pf \
      --ns3-arg enableHarqReTx=true \
      --ns3-arg aequitasEnableMcsSelection=false \
      >"${log}" 2>&1 &
    train_pid=$!
    sleep 2
    MONITOR_TAG="rqrdqn_pf" \
      "${ROOT_DIR}/scratch/rl_bwp/monitor_ns3_reconnector.sh" "${train_pid}" "${PORT_RQR}" 10 5 \
      "${NS3_SCRIPT}" --numUes="${NUM_UES}" --simTime="${SIM_TIME}" --envStepTime="${STEP}" \
      --enableOpenGym=true --trafficModel=mixed --schedulerPolicy=pf \
      --enableHarqReTx=true --aequitasEnableMcsSelection=false &
    monitor_pid=$!
    wait "${train_pid}"
    kill "${monitor_pid}" 2>/dev/null || true
  ) &
}

launch_drqn_aequitas() {
  local log="${LOG_DIR}/drqn_aequitas.log"
  echo "Launch DRQN+Aequitas -> ${log}"
  "${PYTHON}" "${ROOT_DIR}/scratch/rl_bwp/train_drqn.py" \
    --backend ns3 \
    --run-name "appmix_drqn_aequitas_ue20_sim5_step010_ep10_harq" \
    --seed 2001 \
    --num-ues "${NUM_UES}" \
    --episode-time-s "${SIM_TIME}" \
    --step-time-s "${STEP}" \
    --min-completed-episodes "${EPISODES}" \
    --total-env-steps "${TOTAL_ENV_STEPS}" \
    --reward-bin-size 200 \
    --save-every-episodes 1 \
    --start-sim \
    --port "${PORT_DRQN_AEQ}" \
    --ns3-script "${NS3_SCRIPT}" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=aequitas \
    --ns3-arg aequitasEnableMcsSelection=true \
    --ns3-arg enableHarqReTx=true \
    >"${log}" 2>&1 &
}

launch_drqn_pf() {
  local log="${LOG_DIR}/drqn_pf.log"
  echo "Launch DRQN+PF -> ${log}"
  "${PYTHON}" "${ROOT_DIR}/scratch/rl_bwp/train_drqn.py" \
    --backend ns3 \
    --run-name "appmix_drqn_pf_ue20_sim5_step010_ep10_harq" \
    --seed 3001 \
    --num-ues "${NUM_UES}" \
    --episode-time-s "${SIM_TIME}" \
    --step-time-s "${STEP}" \
    --min-completed-episodes "${EPISODES}" \
    --total-env-steps "${TOTAL_ENV_STEPS}" \
    --reward-bin-size 200 \
    --save-every-episodes 1 \
    --start-sim \
    --port "${PORT_DRQN_PF}" \
    --ns3-script "${NS3_SCRIPT}" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=pf \
    --ns3-arg aequitasEnableMcsSelection=false \
    --ns3-arg enableHarqReTx=true \
    >"${log}" 2>&1 &
}

launch_baseline() {
  local bwp="$1"
  local log="${LOG_DIR}/bwp${bwp}_aequitas.log"
  local summary="${BASELINE_DIR}/bwp${bwp}_aequitas_summary.txt"
  echo "Launch BWP${bwp}+Aequitas -> ${log}"
  "${ROOT_DIR}/ns3" run \
    "${NS3_SCRIPT} --numUes=${NUM_UES} --simTime=${SIM_TIME} --envStepTime=${STEP} \
    --enableOpenGym=false --trafficModel=mixed --schedulerPolicy=aequitas \
    --aequitasEnableMcsSelection=true --enableHarqReTx=true --initialBwpId=${bwp} \
    --summaryFile=${summary}" >"${log}" 2>&1
}

main() {
  ensure_optimized
  cleanup_leftovers
  pick_free_ports

  launch_rqrdqn_pf
  pid1=$!
  launch_drqn_aequitas
  pid2=$!
  wait "${pid1}" "${pid2}" || {
    echo "First group failed; stopping."
    cleanup_leftovers
    exit 1
  }

  launch_drqn_pf
  pid3=$!
  launch_baseline 0 &
  pid4=$!
  launch_baseline 1 &
  pid5=$!
  wait "${pid3}" "${pid4}" "${pid5}" || {
    echo "Second group failed."
    cleanup_leftovers
    exit 1
  }
}

main "$@"
