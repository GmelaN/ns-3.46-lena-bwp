#!/usr/bin/env bash
set -euo pipefail

OUT_REL="${1:-appmix_bwp0_aequitas_aoi_trace_$(date +%Y%m%d_%H%M%S)}"
TRACE_UE="${2:-0}"
SIM_TIME="${3:-5}"
NUM_UES="${4:-20}"

OUT_DIR="scratch/rl_bwp/runs/${OUT_REL}"
mkdir -p "$OUT_DIR"

Aoi_TRACE_FILE="$OUT_DIR/aoi_trace_ue${TRACE_UE}.csv"
AOI_STATE_TRACE_FILE="$OUT_DIR/aoi_state_trace_ue${TRACE_UE}.csv"
APP_STATE_TRACE_FILE="$OUT_DIR/app_state_trace_ue${TRACE_UE}.csv"
SUMMARY_FILE="$OUT_DIR/summary.txt"
METRICS_FILE="$OUT_DIR/metrics.csv"
LOG_FILE="$OUT_DIR/run.log"

if ! ./ns3 show profile | grep -q "Build profile: optimized"; then
  ./ns3 configure --build-profile=optimized
fi
./ns3 build

CMD=(
  "./ns3" "run"
  "aoi-prb-urban-appmix --numUes=${NUM_UES} --simTime=${SIM_TIME} --envStepTime=0.01 --enableOpenGym=false --trafficModel=mixed --schedulerPolicy=aequitas --aequitasEnableMcsSelection=true --enableMcsSwitch=false --initialBwpId=0 --segmentDurationS=1.0 --aoiTraceFile=${Aoi_TRACE_FILE} --aoiTraceUe=${TRACE_UE} --aoiStateTraceFile=${AOI_STATE_TRACE_FILE} --aoiStateTraceUe=${TRACE_UE} --appStateTraceFile=${APP_STATE_TRACE_FILE} --appStateTraceUe=${TRACE_UE} --metricsTraceFile=${METRICS_FILE} --summaryFile=${SUMMARY_FILE}"
)

printf 'Output directory: %s\n' "$OUT_DIR" | tee "$LOG_FILE"
printf 'AoI trace: %s\n' "$Aoi_TRACE_FILE" | tee -a "$LOG_FILE"
printf 'Running: %s\n' "${CMD[*]}" | tee -a "$LOG_FILE"
"${CMD[@]}" | tee -a "$LOG_FILE"

printf '\nSaved files:\n%s\n%s\n%s\n%s\n%s\n' "$Aoi_TRACE_FILE" "$AOI_STATE_TRACE_FILE" "$APP_STATE_TRACE_FILE" "$METRICS_FILE" "$SUMMARY_FILE" | tee -a "$LOG_FILE"
