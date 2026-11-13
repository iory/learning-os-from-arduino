#!/bin/sh
# joint (A+B) vs baseline (A alone): 3 seeds each, 8192 envs, 2300 iterations, split over two GPUs.
# Each host runs its 3 jobs one after another (they saturate a GPU, so running them side by side buys nothing),
# detached from the shell: tmux on the remote host, a systemd user unit (or setsid) here.
# The second machine is yours, so name it in the environment:
#   REMOTE       ssh host of the second GPU machine (a ~/.ssh/config alias works)
#   REMOTE_DIR   this directory (rl/genesis) on that host, as a checkout of the same commit
#   REMOTE_PY    the python to run there, from an environment made with `uv sync` (e.g. .venv/bin/python)
#   sh compare_two_stage.sh            # start both
#   sh compare_two_stage.sh extend     # +2300 iterations for all six (resume), then evaluate
#   sh compare_two_stage.sh status     # progress on both
#   sh compare_two_stage.sh fetch      # copy the remote results/logs back here
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
REMOTE=${REMOTE:?set REMOTE to the ssh host of the second GPU machine}
RDIR=${REMOTE_DIR:?set REMOTE_DIR to the rl/genesis directory on $REMOTE}
RPY=${REMOTE_PY:?set REMOTE_PY to the python to run on $REMOTE}
ENVS=8192; IT=2300
LOCAL_JOBS="joint:1 baseline:1 joint:2"
REMOTE_JOBS="baseline:2 joint:3 baseline:3"
EXT_LOCAL="baseline:1 joint:2 joint:1"
EXT_REMOTE="baseline:3 baseline:2 joint:3"
case "${1:-start}" in
extend)   # +$IT more iterations for all six runs, each host continuing its own (the stuck baseline s1/s3 and the late joint s2 first)
  rsync -a "$HERE"/gs_env.py "$HERE"/train.py "$HERE"/run_extend.sh "$REMOTE:$RDIR/"
  ssh $REMOTE "tmux has-session -t quad-extend 2>/dev/null || tmux new-session -d -s quad-extend \
    'cd $RDIR && PY=$RPY sh run_extend.sh $ENVS $IT $EXT_REMOTE 2>&1 | tee -a extend.log'"
  CMD="cd $HERE && PY='uv run python' sh run_extend.sh $ENVS $IT $EXT_LOCAL >> extend.log 2>&1"
  systemd-run --user --unit=quad-extend --collect sh -c "$CMD"
  echo "extending: local [$EXT_LOCAL], $REMOTE [$EXT_REMOTE]" ;;
start)
  rsync -a "$HERE"/gs_env.py "$HERE"/render.py "$HERE"/sim2sim.py "$HERE"/train.py "$HERE"/twostage.py \
        "$HERE"/run_queue.sh "$REMOTE:$RDIR/"
  ssh $REMOTE "tmux has-session -t quad-compare 2>/dev/null || tmux new-session -d -s quad-compare \
    'cd $RDIR && PY=$RPY sh run_queue.sh $ENVS $IT $REMOTE_JOBS 2>&1 | tee -a queue.log'"
  CMD="cd $HERE && PY='uv run python' sh run_queue.sh $ENVS $IT $LOCAL_JOBS >> queue.log 2>&1"
  if systemctl --user is-active --quiet default.target 2>/dev/null; then
    systemd-run --user --unit=quad-compare --collect sh -c "$CMD"
  else
    setsid nohup sh -c "$CMD" >/dev/null 2>&1 &
  fi
  echo "started: local [$LOCAL_JOBS], $REMOTE [$REMOTE_JOBS]" ;;
status)
  echo "== local";  tail -n 3 "$HERE/queue.log" "$HERE/extend.log" 2>/dev/null || true; ls "$HERE/results" 2>/dev/null || true
  echo "== $REMOTE"; ssh $REMOTE "cd $RDIR && tail -n 3 queue.log extend.log; ls results; tmux ls" ;;
fetch)
  rsync -av "$REMOTE:$RDIR/results/" "$HERE/results/"
  rsync -av --include='*/' --include='events.*' --include='model_*.pt' --include='params/**' --exclude='*' \
        "$REMOTE:$RDIR/logs/" "$HERE/logs/" ;;
esac
