#!/usr/bin/env bash
set -euo pipefail

# Sweep association parameters (t-max-diff, t-offset) and summarize metrics.

GT=""
EST=""
OUT_ROOT="/workspace/results"
FORMAT="tum"
DIFFS="0.02,0.05,0.1"
OFFSETS="-1.0,-0.5,0.0,0.5,1.0"
RUN_PREFIX="orbslam3_sweep"

usage() {
  cat <<EOF
Usage:
  $0 --gt <path> --est <path> [options]

Required:
  --gt PATH                 Ground-truth trajectory (TUM)
  --est PATH                Estimated trajectory (TUM)

Options:
  --out-root PATH           Root output directory (default: /workspace/results)
  --format VALUE            Format for evo (default: tum)
  --diffs LIST              Comma-separated t-max-diff values in seconds (default: 0.02,0.05,0.1)
  --offsets LIST            Comma-separated t-offset values in seconds (default: -1.0,-0.5,0.0,0.5,1.0)
  --run-prefix VALUE        Prefix for output folders (default: orbslam3_sweep)
  -h, --help                Show help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gt) GT="$2"; shift 2 ;;
    --est) EST="$2"; shift 2 ;;
    --out-root) OUT_ROOT="$2"; shift 2 ;;
    --format) FORMAT="$2"; shift 2 ;;
    --diffs) DIFFS="$2"; shift 2 ;;
    --offsets) OFFSETS="$2"; shift 2 ;;
    --run-prefix) RUN_PREFIX="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$GT" || -z "$EST" ]]; then
  echo "Error: --gt and --est are required." >&2
  usage
  exit 1
fi

if [[ ! -f "$GT" ]]; then
  echo "Ground-truth not found: $GT" >&2
  exit 1
fi

if [[ ! -f "$EST" ]]; then
  echo "Estimate not found: $EST" >&2
  exit 1
fi

SWEEP_DIR="${OUT_ROOT%/}/${RUN_PREFIX}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$SWEEP_DIR"
CSV="$SWEEP_DIR/summary.csv"

echo "run_dir,t_max_diff,t_offset,matched,total,coverage,ape_rmse,ape_mean,rpe_rmse,rpe_mean" > "$CSV"

IFS=',' read -r -a diff_values <<< "$DIFFS"
IFS=',' read -r -a offset_values <<< "$OFFSETS"

for d in "${diff_values[@]}"; do
  for o in "${offset_values[@]}"; do
    run_name="${RUN_PREFIX}_d${d}_o${o}"
    run_name="${run_name//./p}"
    run_name="${run_name//-/m}"
    echo "==> diff=${d}, offset=${o}"

    bash /workspace/scripts/evaluate_orbslam_vs_gt.sh \
      --gt "$GT" \
      --est "$EST" \
      --format "$FORMAT" \
      --t-max-diff "$d" \
      --t-offset "$o" \
      --no-plot \
      --run-name "$run_name" \
      --out-root "$SWEEP_DIR" >/dev/null

    run_dir="$SWEEP_DIR/$run_name"

    # Extract matching coverage from APE console.
    read -r matched total coverage <<EOF2
$(python3 - "$run_dir/ape_console.txt" <<'PY'
import re,sys
txt=open(sys.argv[1],encoding="utf-8",errors="ignore").read()
m=re.search(r"Found\s+(\d+)\s+of max\.\s+(\d+)\s+possible matching timestamps",txt)
if not m:
    print("0 0 0.0")
else:
    a=int(m.group(1)); b=int(m.group(2))
    c=(a/b) if b else 0.0
    print(a,b,c)
PY
)
EOF2

    read -r ape_rmse ape_mean rpe_rmse rpe_mean <<EOF3
$(python3 - "$run_dir/summary.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1],encoding="utf-8"))
ape=d.get("ape",{})
rpe=d.get("rpe",{})
print(
    ape.get("rmse",""),
    ape.get("mean",""),
    rpe.get("rmse",""),
    rpe.get("mean","")
)
PY
)
EOF3

    echo "${run_dir},${d},${o},${matched},${total},${coverage},${ape_rmse},${ape_mean},${rpe_rmse},${rpe_mean}" >> "$CSV"
  done
done

echo "Sweep completed."
echo "Summary CSV: $CSV"
