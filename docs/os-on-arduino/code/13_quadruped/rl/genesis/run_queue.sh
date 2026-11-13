#!/bin/sh
# Runs "mode:seed" jobs one after another on THIS machine's GPU: train, then evaluate.
#   PY="uv run python" sh run_queue.sh 8192 2300 joint:1 baseline:1 joint:2
# Results: results/<mode>_s<seed>.txt (sim2sim, nominal and --dr); logs: logs/rsl_rl/.../*_<mode>_s<seed>
cd "$(dirname "$0")"
PY=${PY:-uv run python}
ENVS=$1; IT=$2; shift 2
CMDS="0.06,0;0.12,0;0,0.3;0.06,-0.3"
mkdir -p results
for job in "$@"; do
  mode=${job%%:*}; seed=${job##*:}
  name=${mode}_s$seed
  if [ -f results/$name.txt ]; then echo "[queue] $name done, skip"; continue; fi
  echo "[queue] $(date +%T) train $name"
  $PY train.py --mode $mode --seed $seed --iterations $IT --num-envs $ENVS --run-name $name > results/$name.train.log 2>&1
  D=$(ls -d logs/rsl_rl/arduino_quad_velocity/*_$name | tail -1)
  CK=$(ls $D/model_*.pt | sort -V | tail -1)
  { echo "checkpoint: $CK"
    for dr in "" "--dr"; do $PY sim2sim.py --checkpoint $CK --sim genesis --commands "$CMDS" $dr; done
  } > results/$name.txt.tmp 2>&1 && mv results/$name.txt.tmp results/$name.txt
  echo "[queue] $(date +%T) done $name"
done
