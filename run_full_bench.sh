#!/bin/bash
RESULTS_FILE="fixed_bench_results.csv"
echo "UE_ID,BWP,MCS_Offset,Avg_AoI_ms,Throughput_Mbps,BLER,Avg_MCS" > $RESULTS_FILE

# Test only requested 4 cases
for bwp in 0 1; do
    for offset in -2 2; do
        echo "Running: BWP=$bwp, Offset=$offset"
        ./ns3 run "scratch/aoi-prb-urban-appmix --simTime=1.5 --appStart=0.5 --numUes=20 --enableMobility=false --fixedBwp=$bwp --fixedMcsOffset=$offset --appLoadScale=1.0" > temp_log.txt 2>&1
        
        for i in {0..19}; do
            LINE=$(grep -w "UE$i" temp_log.txt | grep "class=")
            THR=$(echo $LINE | awk -F'thr=' '{print $2}' | awk '{print $1}')
            AOI=$(echo $LINE | awk -F'AoI\(node\)=' '{print $2}' | awk '{print $1}')
            MCS=$(echo $LINE | awk -F'avgMcs=' '{print $2}' | awk '{print $1}')
            BLER=$(echo $LINE | awk -F'bler=' '{print $2}' | awk '{print $1}')
            
            echo "$i,$bwp,$offset,$AOI,$THR,$BLER,$MCS" >> $RESULTS_FILE
        done
    done
done

# Summary analysis for representative UEs
echo ""
echo "=== Comparison (BWP 0 vs BWP 1 with MCS offsets) ==="
printf "%-5s | %-5s | %-10s | %-10s | %-10s | %-10s\n" "UE" "BWP" "Offset" "AoI" "Thr" "BLER"
for i in 0 6 12 18; do
    for bwp in 0 1; do
        for offset in -2 2; do
            res=$(grep "^$i,$bwp,$offset," $RESULTS_FILE)
            aoi=$(echo $res | cut -d',' -f4)
            thr=$(echo $res | cut -d',' -f5)
            bler=$(echo $res | cut -d',' -f6)
            printf "%-5d | %-5d | %-10d | %-10s | %-10s | %-10s\n" $i $bwp $offset $aoi $thr $bler
        done
    done
    echo "------------------------------------------------------------------------"
done
