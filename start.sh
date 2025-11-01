#!/bin/bash
export OMP_NUM_THREADS=1

host=${1:-localhost}
port=${2:-3100}

# Pre-build C++ modules once before launching multiple players in parallel.
python3 - <<'PY'
from scripts.commons.Script import Script
Script.build_cpp_modules()
PY

for i in {1..5}; do
  python3 ./Run_Player.py -i $host -p $port -u $i -t sipho -P 0 -D 0 &
  sleep 0.9
done
