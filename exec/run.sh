#!/bin/sh
~/src/ns-4/ns3 && cp ~/src/ns-4/build/scratch/ns3.46-bwp-switch-mac-priority-default ~/src/ns-4/exec/
cd ~/src/ns-4/exec
rm -rf *.out
./ns3.46-bwp-switch-mac-priority-default --simTime=60 --numUes=30 --initialBwp=0 --enableInternalPolicy=true 2> rl.out &
./ns3.46-bwp-switch-mac-priority-default --simTime=60 --numUes=30 --initialBwp=0 --enableInternalPolicy=false 2> 0.out &
./ns3.46-bwp-switch-mac-priority-default --simTime=60 --numUes=30 --initialBwp=1 --enableInternalPolicy=false 2> 1.out &

