#!/usr/bin/env bash
set -euo pipefail

# Robust ORB-SLAM3 launcher for container runs.
# Default mode is headless (xvfb), so it works even without host X11 auth.

VOCAB="/workspace/ORB_SLAM3/Vocabulary/ORBvoc.txt"
CONFIG="/workspace/configs/orbslam_config_headless.yaml"
SEQ_DIR=""
TIMESTAMPS=""
MODE="headless" # headless | gui
OUTPUT_DIR="/workspace/results/orbslam3_latest"

usage() {
  cat <<EOF
Usage:
  $0 --seq-dir <converted_seq_dir> [options]

Required:
  --seq-dir PATH            EuRoC sequence dir (e.g. /workspace/dataset/converted/s3li_traverse_1)

Options:
  --timestamps PATH         Timestamps file (default: <seq-dir>/timestamps.txt)
  --config PATH             ORB-SLAM3 yaml config (default: /workspace/configs/orbslam_config_headless.yaml)
  --vocab PATH              ORB vocabulary (default: /workspace/ORB_SLAM3/Vocabulary/ORBvoc.txt)
  --mode VALUE              headless|gui (default: headless)
  --output-dir PATH         Directory where ORB-SLAM3 writes CameraTrajectory.txt (default: /workspace/results/orbslam3_latest)
  -h, --help                Show help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq-dir) SEQ_DIR="$2"; shift 2 ;;
    --timestamps) TIMESTAMPS="$2"; shift 2 ;;
    --config) CONFIG="$2"; shift 2 ;;
    --vocab) VOCAB="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$SEQ_DIR" ]]; then
  echo "Error: --seq-dir is required." >&2
  usage
  exit 1
fi

if [[ -z "$TIMESTAMPS" ]]; then
  TIMESTAMPS="${SEQ_DIR%/}/timestamps.txt"
fi

if [[ ! -f "$VOCAB" ]]; then
  echo "Vocabulary not found: $VOCAB" >&2
  exit 1
fi
if [[ ! -f "$CONFIG" ]]; then
  echo "Config not found: $CONFIG" >&2
  exit 1
fi
if [[ ! -f "$TIMESTAMPS" ]]; then
  echo "Timestamps not found: $TIMESTAMPS" >&2
  exit 1
fi
if [[ ! -d "$SEQ_DIR" ]]; then
  echo "Sequence directory not found: $SEQ_DIR" >&2
  exit 1
fi

mkdir -p "$OUTPUT_DIR"
cd "$OUTPUT_DIR"

CMD=(/workspace/ORB_SLAM3/Examples/Stereo/stereo_euroc "$VOCAB" "$CONFIG" "$SEQ_DIR" "$TIMESTAMPS")

if [[ "$MODE" == "headless" ]]; then
  xvfb-run -a "${CMD[@]}"
elif [[ "$MODE" == "gui" ]]; then
  "${CMD[@]}"
else
  echo "Invalid --mode: $MODE (use headless|gui)" >&2
  exit 1
fi

echo "Expected outputs:"
echo "  $OUTPUT_DIR/CameraTrajectory.txt"
echo "  $OUTPUT_DIR/KeyFrameTrajectory.txt"
