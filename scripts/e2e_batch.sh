#!/bin/bash
# Run a list of tasks sequentially: e2e_batch.sh <batch-name> "<cmd>|<track models>" ...
cd /home/gaminizer/Projects/ARIA
source /opt/ros/humble/setup.bash; source install/setup.bash
B=$1; shift
i=0
for spec in "$@"; do
  i=$((i+1)); cmd="${spec%%|*}"; trk="${spec#*|}"
  timeout 560 python3 scripts/e2e_task_runner.py "$cmd" --name "${B}_${i}" --track "$trk" --reset --timeout 500 > /tmp/${B}_${i}.log 2>&1
  echo "[$i] $cmd -> $(grep FINAL /tmp/${B}_${i}.log | head -1)  $(grep GROUND /tmp/${B}_${i}.log | head -3 | tr '\n' ' ')"
done
