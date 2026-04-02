#!/bin/bash

/home/jshyeon/src/ns-3.46-bwp-temp/ns3 build
source /home/jshyeon/src/ns-3.46-bwp-temp/ns3gym-venv/bin/activate

./ns3 run 'aoi-prb-urban-appmix --numUes=20 --simTime=5 --envStepTime=0.01 --enableOpenGym=false --trafficModel=mixed --schedulerPolicy=aequitas --aequitasEnableMcsSelection=true --enableMcsSwitch=false --initialBwpId=0 --segmentDurationS=1.0 --aoiStateTraceFile=results/aoi_packet_traces/BWP0+Aequitas/appmix_bwp0_aeq_ue0_5s_aoi_state.csv --aoiStateTraceUe=0 --queueTraceFile=results/aoi_packet_traces/BWP0+Aequitas/appmix_bwp0_aeq_ue0_5s_queue.csv --queueTraceUe=0 --appStateTraceFile=results/aoi_packet_traces/BWP0+Aequitas/appmix_bwp0_aeq_ue0_5s_app_state.csv --appStateTraceUe=0 --aoiTraceFile=results/aoi_packet_traces/BWP0+Aequitas/appmix_bwp0_aeq_ue0_5s_aoi_trace.csv --aoiTraceUe=0 --metricsTraceFile=results/aoi_packet_traces/BWP0+Aequitas/appmix_bwp0_aeq_ue0_5s_metrics.csv --summaryFile=results/aoi_packet_traces/BWP0+Aequitas/appmix_bwp0_aeq_ue0_5s_summary.txt'