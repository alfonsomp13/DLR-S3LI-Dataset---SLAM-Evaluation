#!/usr/bin/env bash
set -euo pipefail

# Reproducible ORB-SLAM3 ablation runner:
# 1) clones a template config,
# 2) applies ORB/stereo parameter overrides,
# 3) runs ORB-SLAM3 per sequence,
# 4) evaluates against GT and writes a summary CSV.

TEMPLATE_CONFIG="/workspace/configs/orbslam_config_headless.yaml"
SEQS="s3li_traverse_1,s3li_traverse_2"
OUT_ROOT="/workspace/results"
RUN_PREFIX="orbslam3_orb_ablation"
T_MAX_DIFF="0.05"
T_OFFSET="0.0"
MODE="headless"

VARIANT_NAME=""
NFEATURES=""
INI_FAST=""
MIN_FAST=""
NLEVELS=""
SCALE_FACTOR=""
TH_DEPTH=""

usage() {
  cat <<EOF
Usage:
  $0 --variant-name NAME [parameter overrides] [options]

Required:
  --variant-name NAME           Variant label used in output folders

ORB/stereo overrides (optional):
  --nfeatures INT               ORBextractor.nFeatures
  --ini-fast INT                ORBextractor.iniThFAST
  --min-fast INT                ORBextractor.minThFAST
  --nlevels INT                 ORBextractor.nLevels
  --scale-factor FLOAT          ORBextractor.scaleFactor
  --th-depth FLOAT              Stereo.ThDepth

Options:
  --template-config PATH        Base config template (default: /workspace/configs/orbslam_config_headless.yaml)
  --seqs LIST                   Comma-separated seq ids (default: s3li_traverse_1,s3li_traverse_2)
  --out-root PATH               Output root (default: /workspace/results)
  --run-prefix NAME             Run folder prefix (default: orbslam3_orb_ablation)
  --t-max-diff FLOAT            Association tolerance (default: 0.05)
  --t-offset FLOAT              Time offset for all seqs (default: 0.0)
  --mode VALUE                  ORB run mode: headless|gui (default: headless)
  -h, --help                    Show help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --variant-name) VARIANT_NAME="$2"; shift 2 ;;
    --nfeatures) NFEATURES="$2"; shift 2 ;;
    --ini-fast) INI_FAST="$2"; shift 2 ;;
    --min-fast) MIN_FAST="$2"; shift 2 ;;
    --nlevels) NLEVELS="$2"; shift 2 ;;
    --scale-factor) SCALE_FACTOR="$2"; shift 2 ;;
    --th-depth) TH_DEPTH="$2"; shift 2 ;;
    --template-config) TEMPLATE_CONFIG="$2"; shift 2 ;;
    --seqs) SEQS="$2"; shift 2 ;;
    --out-root) OUT_ROOT="$2"; shift 2 ;;
    --run-prefix) RUN_PREFIX="$2"; shift 2 ;;
    --t-max-diff) T_MAX_DIFF="$2"; shift 2 ;;
    --t-offset) T_OFFSET="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$VARIANT_NAME" ]]; then
  echo "Error: --variant-name is required." >&2
  usage
  exit 1
fi

if [[ ! -f "$TEMPLATE_CONFIG" ]]; then
  echo "Template config not found: $TEMPLATE_CONFIG" >&2
  exit 1
fi

RUN_DIR="${OUT_ROOT%/}/${RUN_PREFIX}_$(date +%Y%m%d_%H%M%S)_${VARIANT_NAME}"
mkdir -p "$RUN_DIR"
CONFIG_OUT="$RUN_DIR/orbslam_config_${VARIANT_NAME}.yaml"
cp "$TEMPLATE_CONFIG" "$CONFIG_OUT"

python3 - "$CONFIG_OUT" "$NFEATURES" "$INI_FAST" "$MIN_FAST" "$NLEVELS" "$SCALE_FACTOR" "$TH_DEPTH" <<'PY'
import re
import sys

config_path = sys.argv[1]
nfeatures, ini_fast, min_fast, nlevels, scale_factor, th_depth = sys.argv[2:]

updates = {}
if nfeatures:
    updates["ORBextractor.nFeatures"] = nfeatures
if ini_fast:
    updates["ORBextractor.iniThFAST"] = ini_fast
if min_fast:
    updates["ORBextractor.minThFAST"] = min_fast
if nlevels:
    updates["ORBextractor.nLevels"] = nlevels
if scale_factor:
    updates["ORBextractor.scaleFactor"] = scale_factor
if th_depth:
    updates["Stereo.ThDepth"] = th_depth

with open(config_path, "r", encoding="utf-8") as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    for key, val in updates.items():
        if re.match(rf"^\s*{re.escape(key)}\s*:", line):
            lines[i] = f"{key}: {val}\n"

with open(config_path, "w", encoding="utf-8") as f:
    f.writelines(lines)

print("Applied overrides:")
for k, v in updates.items():
    print(f"  {k}={v}")
PY

CSV="$RUN_DIR/summary.csv"
echo "variant,sequence,t_max_diff,t_offset,matched,total,coverage,ape_rmse,ape_mean,rpe_rmse,rpe_mean,run_dir,eval_dir,config" > "$CSV"

IFS=',' read -r -a seq_array <<< "$SEQS"
for seq in "${seq_array[@]}"; do
  seq="$(echo "$seq" | xargs)"
  seq_dir="/workspace/dataset/converted/${seq}"
  gt_tum="/workspace/results/${seq}_gt_tum.txt"
  orb_out="$RUN_DIR/orb_${seq}"
  eval_name="eval_${seq}_${VARIANT_NAME}"

  if [[ ! -d "$seq_dir" ]]; then
    echo "Missing sequence dir: $seq_dir" >&2
    exit 1
  fi
  if [[ ! -f "$gt_tum" ]]; then
    echo "Missing GT TUM file: $gt_tum" >&2
    exit 1
  fi

  echo "==> Running ORB-SLAM3 (${seq})"
  set +e
  bash /workspace/scripts/run_orbslam3_stereo.sh \
    --seq-dir "$seq_dir" \
    --config "$CONFIG_OUT" \
    --mode "$MODE" \
    --output-dir "$orb_out" \
    > "$orb_out.run_console.txt" 2>&1
  orb_exit=$?
  set -e

  est_traj="$orb_out/CameraTrajectory.txt"
  if [[ ! -f "$est_traj" ]]; then
    echo "ORB-SLAM3 failed for ${seq} (exit=${orb_exit}) and no trajectory was produced: $est_traj" >&2
    exit 1
  fi
  if [[ "$orb_exit" -ne 0 ]]; then
    echo "WARN: ORB-SLAM3 exited with code ${orb_exit} for ${seq}, continuing because trajectory exists."
  fi

  echo "==> Evaluating (${seq})"
  bash /workspace/scripts/evaluate_orbslam_vs_gt.sh \
    --gt "$gt_tum" \
    --est "$est_traj" \
    --format tum \
    --t-max-diff "$T_MAX_DIFF" \
    --t-offset "$T_OFFSET" \
    --run-name "$eval_name" \
    --out-root "$RUN_DIR" \
    --no-plot \
    > "$orb_out.eval_console.txt" 2>&1

  eval_dir="$RUN_DIR/$eval_name"

  read -r matched total coverage <<EOF2
$(python3 - "$eval_dir/ape_console.txt" <<'PY'
import re
import sys
txt=open(sys.argv[1], encoding="utf-8", errors="ignore").read()
m=re.search(r"Found\s+(\d+)\s+of max\.\s+(\d+)\s+possible matching timestamps", txt)
if not m:
    print("0 0 0.0")
else:
    a=int(m.group(1)); b=int(m.group(2)); c=(a/b) if b else 0.0
    print(a, b, c)
PY
)
EOF2

  read -r ape_rmse ape_mean rpe_rmse rpe_mean <<EOF3
$(python3 - "$eval_dir/summary.json" <<'PY'
import json
import sys
d=json.load(open(sys.argv[1], encoding="utf-8"))
ape=d.get("ape", {})
rpe=d.get("rpe", {})
print(ape.get("rmse",""), ape.get("mean",""), rpe.get("rmse",""), rpe.get("mean",""))
PY
)
EOF3

  echo "${VARIANT_NAME},${seq},${T_MAX_DIFF},${T_OFFSET},${matched},${total},${coverage},${ape_rmse},${ape_mean},${rpe_rmse},${rpe_mean},${orb_out},${eval_dir},${CONFIG_OUT}" >> "$CSV"
done

echo "Ablation run completed."
echo "Run dir: $RUN_DIR"
echo "Summary: $CSV"
