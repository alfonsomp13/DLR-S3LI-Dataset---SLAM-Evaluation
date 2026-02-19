#!/usr/bin/env bash
set -euo pipefail

# Evaluate ORB-SLAM3 trajectory against ground truth using evo.
# Outputs all artifacts under /workspace/results by default.

FORMAT="tum"
GT_PATH=""
EST_PATH=""
OUT_ROOT="/workspace/results"
RUN_NAME="orbslam3_eval_$(date +%Y%m%d_%H%M%S)"
DELTA="1"
DELTA_UNIT="f"
TMAXDIFF=""
TOFFSET="0.0"
PLOT="0"
PLOT_BACKEND="Agg"

usage() {
  cat <<EOF
Usage:
  $0 --gt <groundtruth.txt> --est <estimated.txt> [options]

Required:
  --gt PATH                 Ground-truth trajectory file
  --est PATH                Estimated trajectory file (e.g. CameraTrajectory.txt)

Options:
  --format VALUE            Trajectory format for both files (default: tum)
                            Valid values: tum | euroc | kitti | bag
  --out-root PATH           Root output directory (default: /workspace/results)
  --run-name NAME           Output folder name (default: timestamp-based)
  --delta VALUE             RPE delta value (default: 1)
  --delta-unit VALUE        RPE delta unit: f|d|r|m (default: f)
  --t-max-diff VALUE        Max timestamp diff for association (seconds)
  --t-offset VALUE          Constant timestamp offset added to estimate (seconds)
  --plot                    Enable plotting (requires a valid matplotlib backend)
  --no-plot                 Disable plotting (default, faster and headless-safe)
  --plot-backend VALUE      Matplotlib backend for evo when plotting (default: Agg)
  -h, --help                Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gt) GT_PATH="$2"; shift 2 ;;
    --est) EST_PATH="$2"; shift 2 ;;
    --format) FORMAT="$2"; shift 2 ;;
    --out-root) OUT_ROOT="$2"; shift 2 ;;
    --run-name) RUN_NAME="$2"; shift 2 ;;
    --delta) DELTA="$2"; shift 2 ;;
    --delta-unit) DELTA_UNIT="$2"; shift 2 ;;
    --t-max-diff) TMAXDIFF="$2"; shift 2 ;;
    --t-offset) TOFFSET="$2"; shift 2 ;;
    --plot) PLOT="1"; shift 1 ;;
    --no-plot) PLOT="0"; shift 1 ;;
    --plot-backend) PLOT_BACKEND="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$GT_PATH" || -z "$EST_PATH" ]]; then
  echo "Error: --gt and --est are required." >&2
  usage
  exit 1
fi

if [[ ! -f "$GT_PATH" ]]; then
  echo "Ground truth file not found: $GT_PATH" >&2
  exit 1
fi

if [[ ! -f "$EST_PATH" ]]; then
  echo "Estimated trajectory file not found: $EST_PATH" >&2
  exit 1
fi

if ! command -v evo_ape >/dev/null 2>&1 || ! command -v evo_rpe >/dev/null 2>&1 || ! command -v evo_traj >/dev/null 2>&1; then
  echo "evo is not installed in this environment. Rebuild the Docker image." >&2
  exit 1
fi

if [[ "$PLOT" == "1" ]]; then
  # Force a headless-safe backend in evo settings to avoid TkAgg/X11 crashes.
  python3 - "$PLOT_BACKEND" <<'PY'
import json
import os
import sys

backend = sys.argv[1]
settings_dir = os.path.join(os.path.expanduser("~"), ".evo")
os.makedirs(settings_dir, exist_ok=True)
settings_path = os.path.join(settings_dir, "settings.json")

data = {}
if os.path.exists(settings_path):
    try:
        with open(settings_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        data = {}

data["plot_backend"] = backend
data["plot_seaborn_enabled"] = True
data["plot_usetex"] = False

with open(settings_path, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=2, sort_keys=True)

print(f"Configured evo plot backend: {backend} ({settings_path})")
PY
fi

OUT_DIR="${OUT_ROOT%/}/${RUN_NAME}"
mkdir -p "$OUT_DIR"

ASSOC_ARGS=()
if [[ -n "$TMAXDIFF" ]]; then
  ASSOC_ARGS+=(--t_max_diff "$TMAXDIFF")
fi
ASSOC_ARGS+=(--t_offset "$TOFFSET")

echo "==> Writing evaluation outputs to: $OUT_DIR"
cp "$GT_PATH" "$OUT_DIR/groundtruth_input.txt"
cp "$EST_PATH" "$OUT_DIR/estimate_input.txt"

GT_EVAL="$GT_PATH"
EST_EVAL="$EST_PATH"

# ORB-SLAM3 often writes TUM trajectories with timestamps in nanoseconds.
# evo expects TUM timestamps in seconds. Normalize automatically when needed.
if [[ "$FORMAT" == "tum" ]]; then
  python3 - "$GT_PATH" "$OUT_DIR/groundtruth_eval_tum.txt" <<'PY'
import sys
src, dst = sys.argv[1], sys.argv[2]
rows = []
with open(src, "r", encoding="utf-8", errors="ignore") as f:
    for line in f:
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        p = s.split()
        if len(p) < 8:
            continue
        t = float(p[0])
        if abs(t) > 1e12:
            t /= 1e9
        rows.append(f"{t:.9f} {' '.join(p[1:8])}\n")
with open(dst, "w", encoding="utf-8") as f:
    f.writelines(rows)
print(f"normalized {len(rows)} poses -> {dst}")
PY

  python3 - "$EST_PATH" "$OUT_DIR/estimate_eval_tum.txt" <<'PY'
import sys
src, dst = sys.argv[1], sys.argv[2]
rows = []
with open(src, "r", encoding="utf-8", errors="ignore") as f:
    for line in f:
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        p = s.split()
        if len(p) < 8:
            continue
        t = float(p[0])
        if abs(t) > 1e12:
            t /= 1e9
        rows.append(f"{t:.9f} {' '.join(p[1:8])}\n")
with open(dst, "w", encoding="utf-8") as f:
    f.writelines(rows)
print(f"normalized {len(rows)} poses -> {dst}")
PY

  GT_EVAL="$OUT_DIR/groundtruth_eval_tum.txt"
  EST_EVAL="$OUT_DIR/estimate_eval_tum.txt"
fi

echo "==> Running APE..."
if [[ "$PLOT" == "1" ]]; then
  evo_ape "$FORMAT" "$GT_EVAL" "$EST_EVAL" \
    --align \
    --verbose \
    --plot \
    --save_plot "$OUT_DIR/ape_plot.pdf" \
    --save_results "$OUT_DIR/ape.zip" \
    "${ASSOC_ARGS[@]}" \
    | tee "$OUT_DIR/ape_console.txt"
else
  evo_ape "$FORMAT" "$GT_EVAL" "$EST_EVAL" \
    --align \
    --verbose \
    --save_results "$OUT_DIR/ape.zip" \
    "${ASSOC_ARGS[@]}" \
    | tee "$OUT_DIR/ape_console.txt"
fi

echo "==> Running RPE..."
if [[ "$PLOT" == "1" ]]; then
  evo_rpe "$FORMAT" "$GT_EVAL" "$EST_EVAL" \
    --align \
    --verbose \
    --delta "$DELTA" \
    --delta_unit "$DELTA_UNIT" \
    --plot \
    --save_plot "$OUT_DIR/rpe_plot.pdf" \
    --save_results "$OUT_DIR/rpe.zip" \
    "${ASSOC_ARGS[@]}" \
    | tee "$OUT_DIR/rpe_console.txt"
else
  evo_rpe "$FORMAT" "$GT_EVAL" "$EST_EVAL" \
    --align \
    --verbose \
    --delta "$DELTA" \
    --delta_unit "$DELTA_UNIT" \
    --save_results "$OUT_DIR/rpe.zip" \
    "${ASSOC_ARGS[@]}" \
    | tee "$OUT_DIR/rpe_console.txt"
fi

echo "==> Saving trajectory overlay plot..."
if [[ "$PLOT" == "1" ]]; then
  evo_traj "$FORMAT" "$GT_EVAL" "$EST_EVAL" \
    --ref="$GT_EVAL" \
    --align \
    --plot \
    --save_plot "$OUT_DIR/traj_overlay.pdf" \
    "${ASSOC_ARGS[@]}" \
    | tee "$OUT_DIR/traj_console.txt"
else
  echo "plot disabled (--no-plot)" | tee "$OUT_DIR/traj_console.txt"
fi

echo "==> Exporting quick summary..."
python3 - "$OUT_DIR" <<'PY'
import json
import os
import sys
import zipfile

out_dir = sys.argv[1]
summary = {}

def load_stats(zip_name, key):
    zip_path = os.path.join(out_dir, zip_name)
    if not os.path.exists(zip_path):
        return
    with zipfile.ZipFile(zip_path) as zf:
        with zf.open("stats.json") as f:
            stats = json.load(f)
    summary[key] = {
        "rmse": stats.get("rmse"),
        "mean": stats.get("mean"),
        "median": stats.get("median"),
        "std": stats.get("std"),
        "min": stats.get("min"),
        "max": stats.get("max"),
        "sse": stats.get("sse"),
    }

load_stats("ape.zip", "ape")
load_stats("rpe.zip", "rpe")

with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2)

print(json.dumps(summary, indent=2))
PY

echo "==> Done."
echo "Results available at: $OUT_DIR"
