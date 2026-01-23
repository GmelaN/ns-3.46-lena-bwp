#!/bin/sh
rm /home/jshyeon/src/ns-4/build/scratch/ns3.46-bwp-switch-mac-priority-default
rm /home/jshyeon/src/ns-4/build/scratch/ns3.46-bwp-switch-mac-priority-periodic-default
/home/jshyeon/src/ns-4/ns3 build
/home/jshyeon/src/ns-4/build/scratch/ns3.46-bwp-switch-mac-priority-default --simTime=10 --numUes=30 --initialBwp=0 --enableQlearning=false 2> queue_window_bursty.out &
/home/jshyeon/src/ns-4/build/scratch/ns3.46-bwp-switch-mac-priority-default --simTime=10 --numUes=30 --initialBwp=0 --enableQlearning=true 2> q_learning_bursty.out &
/home/jshyeon/src/ns-4/build/scratch/ns3.46-bwp-switch-mac-priority-periodic-default --simTime=10 --numUes=30 --initialBwp=0 --enableQlearning=false 2> queue_window_periodic.out &
/home/jshyeon/src/ns-4/build/scratch/ns3.46-bwp-switch-mac-priority-periodic-default --simTime=10 --numUes=30 --initialBwp=0 --enableQlearning=true 2> q_learning_periodic.out &
