#!/bin/bash
RESULTS_FILE="bench_results.csv"
echo "BWP,MCS_Offset,Avg_AoI_ms,Throughput_Mbps,BLER,Avg_MCS" > $RESULTS_FILE

# Test coordinates: (60, 0) - Directly behind the building at (40, 0)
UE_X=60.0
UE_Y=0.0

for bwp in 0 1; do
    for offset in -2 -1 0 1 2; do
        echo "Running: BWP=$bwp, Offset=$offset"
        # Run 1 second simulation for quick benchmarking
        ./ns3 run "scratch/aoi-prb-urban-appmix --simTime=1.3 --appStart=0.3 --numUes=5 --fixedUeX=$UE_X --fixedUeY=$UE_Y --enableMobility=false --fixedBwp=$bwp --fixedMcsOffset=$offset" > temp_log.txt 2>&1
        
        # Extract UE0 results from the log (using grep and awk)
        LINE=$(grep "UE0 class=" temp_log.txt)
        THR=$(echo $LINE | awk -F'thr=' '{print $2}' | awk '{print $1}')
        AOI=$(echo $LINE | awk -F'AoI\(node\)=' '{print $2}' | awk '{print $1}')
        MCS=$(echo $LINE | awk -F'avgMcs=' '{print $2}' | awk '{print $1}')
        BLER=$(echo $LINE | awk -F'bler=' '{print $2}' | awk '{print $1}')
        
        echo "$bwp,$offset,$AOI,$THR,$BLER,$MCS" >> $RESULTS_FILE
    done
done
cat $RESULTS_FILE
