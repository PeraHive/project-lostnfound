#!/bin/bash

# Source the virtual environment
source /home/jazz/venv-ardupilot/bin/activate

# Start ArduPilot SITL
sim_vehicle.py -v ArduCopter -f quad \
  --custom-location=7.252591,80.592751,0,0 \
  --out=127.0.0.1:14550 \
  --out=127.0.0.1:14551
  