#!/usr/bin/env bash
# Record the README GIFs from the fixed world camera, headless, and check their size and length.
#   scripts/record_gifs.sh   -> docs/assets/hero.gif, tracking.gif, jogging.gif
# On the shared dev laptop run it as `flock ~/work/.locks/heavy.lock scripts/record_gifs.sh`
# (scripts/lib/sim_session.sh takes the Gazebo lock itself).
#
# Gazebo renders the camera in worlds/bench_camera.sdf with `gz sim -s --headless-rendering` (EGL).
# record_mode (ur5e_teaching_bench) drives each scene and saves the raw frames with the joint
# states, the scripted push, the controller reference and the jog state; scripts/compose_gifs.py
# draws the panels, the force arrow, the tool-point traces, the captions and the live numbers from
# that data; ffmpeg makes the GIFs. Two sims: the hero needs a 1 kg payload in Gazebo
# (payload_mass:=1.0), the tracking and jogging takes run without one, as their benchmarks do.
# Exits 1 if a GIF is not 640 px wide, is larger than 3 MB or plays longer than 15 s.
#   RAW_DIR=/tmp/gif_raw scripts/record_gifs.sh                  # keep the raw takes there
#   RAW_DIR=/tmp/gif_raw COMPOSE_ONLY=1 scripts/record_gifs.sh   # re-compose them, no Gazebo
set -euo pipefail
cd "$(dirname "$0")/.."
OUT="${OUT:-docs/assets}"
FPS="${FPS:-10}"
MAX_BYTES=$((3 * 1024 * 1024))
MAX_SECONDS=15
WIDTH=640
GIFS=(hero tracking jogging)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}" GZ_PARTITION="${GZ_PARTITION:-r1}"
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

TMP="$(mktemp -d)"
RAW="${RAW_DIR:-${TMP}/raw}"
export SIM_STATE_DIR="${TMP}/sim"  # private: never touch another sim's state file
trap 'scripts/lib/sim_session.sh stop >/dev/null 2>&1 || true; rm -rf "${TMP}"' EXIT
mkdir -p "${OUT}"

record() {  # record <actual payload kg> <scene>...
  local payload="$1"
  shift
  scripts/lib/sim_session.sh start world:=bench_camera.sdf headless_rendering:=true \
    "payload_mass:=${payload}"
  for scene in "$@"; do
    rm -rf "${RAW:?}/${scene}"
    ros2 run ur5e_teaching_bench record_mode --scene "${scene}" --out-dir "${RAW}/${scene}"
  done
  scripts/lib/sim_session.sh stop
}
if [ -z "${COMPOSE_ONLY:-}" ]; then
  record 1.0 hero
  record 0.0 tracking jogging
fi

python3 scripts/compose_gifs.py --raw "${RAW}" --out "${TMP}/frames" --fps "${FPS}"

encode() {  # encode <frame dir> <gif> [colors]
  local colors="${3:-128}"
  ffmpeg -loglevel error -y -framerate "${FPS}" -i "$1/%05d.png" -vf \
    "split[a][b];[a]palettegen=max_colors=${colors}:stats_mode=full[p];[b][p]paletteuse=dither=sierra2_4a:diff_mode=rectangle" \
    -loop 0 "$2"
  if [ "$(stat -c %s "$2")" -gt "${MAX_BYTES}" ] && [ "${colors}" -gt 32 ]; then
    encode "$1" "$2" $((colors / 2))
  fi
}

status=0
for name in "${GIFS[@]}"; do
  gif="${OUT}/${name}.gif"
  encode "${TMP}/frames/${name}" "${gif}"
  size=$(stat -c %s "${gif}")
  width=$(ffprobe -v error -select_streams v:0 -show_entries stream=width -of csv=p=0 "${gif}")
  frames=$(ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames \
    -of csv=p=0 "${gif}")
  duration=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "${gif}")
  echo "[record_gifs] ${gif}: ${size} bytes, ${width} px wide, ${frames} frames, ${duration} s"
  if [ "${width}" -ne "${WIDTH}" ]; then
    echo "[record_gifs] ${gif} is ${width} px wide, not ${WIDTH}" >&2
    status=1
  fi
  if [ "${size}" -gt "${MAX_BYTES}" ]; then
    echo "[record_gifs] ${gif} is larger than 3 MB" >&2
    status=1
  fi
  if ! python3 -c "import sys; sys.exit(0 if float('${duration}') <= ${MAX_SECONDS} else 1)"; then
    echo "[record_gifs] ${gif} plays longer than ${MAX_SECONDS} s" >&2
    status=1
  fi
done
cat "${TMP}/frames/captions.json"
exit "${status}"
