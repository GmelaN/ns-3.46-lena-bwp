#!/bin/sh
rm /home/jshyeon/src/ns-4/build/scratch/ns3.46-bwp-switch-mac-priority-default
/home/jshyeon/src/ns-4/ns3
/home/jshyeon/src/ns-4/build/scratch/ns3.46-bwp-switch-mac-priority-default --simTime=300 --numUes=30 --initialBwp=0 --enableQlearning=false 2> queue_window.out &
/home/jshyeon/src/ns-4/build/scratch/ns3.46-bwp-switch-mac-priority-default --simTime=300 --numUes=30 --initialBwp=0 --enableQlearning=true 2> q_learning.out &
