#!/usr/bin/env bash
# List other heavy processes on this machine and exit 1 if there are any; exit 0 if it is quiet.
#
#   scripts/lib/heavy_processes.sh          # used by regenerate_all.sh and bench_dynamics_timing.sh
#
# Why: the dynamics timing table is a wall-clock measurement, and published numbers should not
# depend on whatever else the machine is doing. (An earlier version of this comment blamed CPU
# load for run-to-run changes in the feedforward tracking RMSE; the recorded runs do not support
# that, see docs/what_broke.md, "Timing offset between reference and state".) Both scripts
# therefore refuse to start next to a simulator, a build, a pip install or a training/eval run.
# The machine-wide locks (~/work/.locks) keep cooperating jobs out; this check catches whatever
# does not take them. It does not look at load average: an editor or browser in the
# background is recorded by the benchmark, not refused.
set -euo pipefail

# Extended regexes over full command lines (the bracket keeps pgrep from matching this script).
patterns=(
  '[g]z sim'
  '[c]olcon (build|test)'
  '[c]c1plus'
  '[p]ip3? install'
  '[m]ove_group'
  '[b]ench_(direct|planners)'
  '[p]ython[0-9.]* .*(lerobot|[^a-z_]train|train\.py|eval\.py)'
)
found=0
seen=" "
for p in "${patterns[@]}"; do
  out=$(pgrep -af -- "${p}" || true)
  while IFS= read -r line; do
    [ -n "${line}" ] || continue
    pid=${line%% *}
    cmd=${line#* }
    # Skip this process tree and shell/flock wrappers (the heavy child itself is listed anyway).
    if [ "${pid}" = "$$" ] || [ "${pid}" = "${PPID}" ]; then continue; fi
    case "${cmd}" in
      bash\ * | /bin/bash\ * | sh\ * | /bin/sh\ * | flock\ *) continue ;;
    esac
    case "${seen}" in *" ${pid} "*) continue ;; esac
    seen="${seen}${pid} "
    echo "heavy process: ${pid} ${cmd:0:160}"
    found=1
  done <<<"${out}"
done
exit "${found}"
