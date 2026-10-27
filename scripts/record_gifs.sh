#!/usr/bin/env bash
# Record the mode GIFs and the hero GIF from the fixed world camera, headless.
# Gazebo renders with `gz sim -s --headless-rendering` (EGL; verified on this laptop, see
# SHARED.md), frames are overlaid with the mode name and a live metric, ffmpeg makes the GIFs.
#   scripts/record_gifs.sh   -> docs/assets/mode_track.gif, mode_handguide.gif, hero.gif
set -euo pipefail
cd "$(dirname "$0")/.."
OUT="${OUT:-docs/assets}"
SECONDS_PER_MODE="${SECONDS_PER_MODE:-10}"
HERO_SECONDS_PER_MODE="${HERO_SECONDS_PER_MODE:-7}"
FPS=10
MAX_BYTES=$((3 * 1024 * 1024))
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}" GZ_PARTITION="${GZ_PARTITION:-r1}"
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

TMP="$(mktemp -d)"
trap 'scripts/lib/sim_session.sh stop; rm -rf "${TMP}"' EXIT
mkdir -p "${OUT}"

scripts/lib/sim_session.sh start world:=bench_camera.sdf headless_rendering:=true payload_mass:=1.0
for mode in track handguide; do
  ros2 run ur5e_teaching_bench record_mode --mode "${mode}" --out-dir "${TMP}/${mode}" \
    --duration "${SECONDS_PER_MODE}" --fps "${FPS}"
done
scripts/lib/sim_session.sh stop

encode() {  # encode <frame dir> <gif> [colors]
  local colors="${3:-128}"
  ffmpeg -loglevel error -y -framerate "${FPS}" -i "$1/%05d.png" -vf \
    "scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=${colors}:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" \
    "$2"
  if [ "$(stat -c %s "$2")" -gt "${MAX_BYTES}" ] && [ "${colors}" -gt 32 ]; then
    encode "$1" "$2" $((colors / 2))
  fi
}

mkdir -p "${TMP}/hero"
n=0
for mode in track handguide; do
  encode "${TMP}/${mode}" "${OUT}/mode_${mode}.gif"
  for f in $(find "${TMP}/${mode}" -name '*.png' | sort | head -n $((HERO_SECONDS_PER_MODE * FPS))); do
    cp "${f}" "${TMP}/hero/$(printf %05d ${n}).png"
    n=$((n + 1))
  done
done
encode "${TMP}/hero" "${OUT}/hero.gif"

for gif in "${OUT}"/mode_track.gif "${OUT}"/mode_handguide.gif "${OUT}"/hero.gif; do
  size=$(stat -c %s "${gif}")
  frames=$(ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames \
    -of csv=p=0 "${gif}")
  echo "[record_gifs] ${gif}: ${size} bytes, ${frames} frames ($((frames / FPS)) s at ${FPS} fps)"
  if [ "${size}" -gt "${MAX_BYTES}" ]; then
    echo "[record_gifs] ${gif} is larger than 3 MB" >&2
    exit 1
  fi
done
