#!/usr/bin/env bash
# List other heavy processes on this machine and exit 1 if there are any; exit 0 if it is quiet.
#
#   scripts/lib/heavy_processes.sh          # used by regenerate_all.sh and bench_dynamics_timing.sh
#
# Why: the published feedforward tracking RMSE tripled when another session's benchmark ran at the
# same time (NOTES.md, 28 Oct), and the dynamics timing table is a wall-clock measurement. Both
# scripts therefore refuse to start next to a simulator, a build, a pip install or a training/eval
# run. The machine-wide locks (~/work/.locks) keep cooperating sessions out; this check catches
# whatever does not take them. It does not look at load average: an editor or browser in the
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
