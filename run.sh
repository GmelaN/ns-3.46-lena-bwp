#!/bin/bash

SIM_TIME=1800
NUM_UES_LIST=(5 10 15 20)

pkill ns3.46-scenario
rm /home/jshyeon/src/ns-4/build/scratch/ns3.46-scenario_1_ftp-default
rm /home/jshyeon/src/ns-4/build/scratch/ns3.46-scenario_2_periodic_burst-default

/home/jshyeon/src/ns-4/ns3 build

for NUM_UES in "${NUM_UES_LIST[@]}"
do
nohup /home/jshyeon/src/ns-4/build/scratch/ns3.46-scenario_1_ftp-default --simTime=${SIM_TIME} --enableMixedTraffic=true --numUes=${NUM_UES} --initialBwp=0 --enableQlearning=false 2> queue_window_bursty_${NUM_UES}.out &
nohup /home/jshyeon/src/ns-4/build/scratch/ns3.46-scenario_1_ftp-default --simTime=${SIM_TIME} --enableMixedTraffic=true --numUes=${NUM_UES} --initialBwp=0 --enableQlearning=true 2> q_learning_bursty_${NUM_UES}.out &
nohup /home/jshyeon/src/ns-4/build/scratch/ns3.46-scenario_1_ftp-default --simTime=${SIM_TIME} --enableMixedTraffic=true --numUes=${NUM_UES} --initialBwp=0 --fixedBwp=true --enableQlearning=false 2> bwp0_bursty_${NUM_UES}.out &
nohup /home/jshyeon/src/ns-4/build/scratch/ns3.46-scenario_1_ftp-default --simTime=${SIM_TIME} --enableMixedTraffic=true --numUes=${NUM_UES} --initialBwp=1 --fixedBwp=true --enableQlearning=false 2> bwp1_bursty_${NUM_UES}.out &


nohup /home/jshyeon/src/ns-4/build/scratch/ns3.46-scenario_2_periodic_burst-default --simTime=${SIM_TIME} --numUes=${NUM_UES} --initialBwp=0 --enableQlearning=false 2> queue_window_periodic_${NUM_UES}.out &
nohup /home/jshyeon/src/ns-4/build/scratch/ns3.46-scenario_2_periodic_burst-default --simTime=${SIM_TIME} --numUes=${NUM_UES} --initialBwp=0 --enableQlearning=true 2> q_learning_periodic_${NUM_UES}.out &
nohup /home/jshyeon/src/ns-4/build/scratch/ns3.46-scenario_2_periodic_burst-default --simTime=${SIM_TIME} --numUes=${NUM_UES} --initialBwp=0  --fixedBwp=true --enableQlearning=false 2> bwp0_periodic_${NUM_UES}.out &
nohup /home/jshyeon/src/ns-4/build/scratch/ns3.46-scenario_2_periodic_burst-default --simTime=${SIM_TIME} --numUes=${NUM_UES} --initialBwp=1  --fixedBwp=true --enableQlearning=false 2> bwp1_periodic_${NUM_UES}.out &
done