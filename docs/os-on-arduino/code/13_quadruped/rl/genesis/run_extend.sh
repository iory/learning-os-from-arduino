#!/bin/sh
# Continues finished "mode:seed" runs for more iterations from their last checkpoint (--resume; the command
# curriculum is restored), then evaluates them like run_queue.sh.
#   PY="uv run python" sh run_extend.sh 8192 2300 baseline:1 joint:2
# Results: results/<mode>_s<seed>_ext.txt / .train.log ; the run: logs/.../*_<mode>_s<seed>_ext
cd "$(dirname "$0")"
PY=${PY:-uv run python}
ENVS=$1; IT=$2; shift 2
CMDS="0.06,0;0.12,0;0,0.3;0.06,-0.3"
mkdir -p results
for job in "$@"; do
  mode=${job%%:*}; seed=${job##*:}
  name=${mode}_s$seed; ext=${name}_ext
  if [ -f results/$ext.txt ]; then echo "[extend] $ext done, skip"; continue; fi
  D0=$(ls -d logs/rsl_rl/arduino_quad_velocity/*_$name | tail -1)
  CK0=$(ls $D0/model_*.pt | sort -V | tail -1)
  echo "[extend] $(date +%T) train $ext from $CK0"
  $PY train.py --mode $mode --seed $seed --iterations $IT --num-envs $ENVS --run-name $ext --resume $CK0 > results/$ext.train.log 2>&1
  D=$(ls -d logs/rsl_rl/arduino_quad_velocity/*_$ext | tail -1)
  CK=$(ls $D/model_*.pt | sort -V | tail -1)
  { echo "checkpoint: $CK"
    for dr in "" "--dr"; do $PY sim2sim.py --checkpoint $CK --sim genesis --commands "$CMDS" $dr; done
  } > results/$ext.txt.tmp 2>&1 && mv results/$ext.txt.tmp results/$ext.txt
  echo "[extend] $(date +%T) done $ext"
done
