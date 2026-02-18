#!/usr/bin/env python3
"""Convert S3LI RTK .pos files to TUM trajectory format.

Supported input:
- baseline_xyz.pos (uses e/n/u-baseline columns as x/y/z)
- global_lle.pos (uses lat/lon/height as x/y/z; mostly for reference)
"""

import argparse
from datetime import datetime, timezone, timedelta


def parse_args():
    parser = argparse.ArgumentParser(description="Convert RTK .pos file to TUM format")
    parser.add_argument("--in-pos", required=True, help="Input .pos file")
    parser.add_argument("--out-tum", required=True, help="Output TUM file")
    parser.add_argument(
        "--mode",
        choices=["baseline_xyz", "global_lle"],
        default="baseline_xyz",
        help="Column mapping mode (default: baseline_xyz)",
    )
    parser.add_argument(
        "--gps-utc-offset",
        type=float,
        default=18.0,
        help="Seconds to subtract from GPST to get UTC epoch (default: 18)",
    )
    return parser.parse_args()


def to_unix_ts(date_str: str, time_str: str, gps_utc_offset: float) -> float:
    dt = datetime.strptime(f"{date_str} {time_str}", "%Y/%m/%d %H:%M:%S.%f")
    dt = dt.replace(tzinfo=timezone.utc) - timedelta(seconds=gps_utc_offset)
    return dt.timestamp()


def main():
    args = parse_args()
    out_lines = []

    with open(args.in_pos, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 6:
                continue

            date_s, time_s = parts[0], parts[1]
            ts = to_unix_ts(date_s, time_s, args.gps_utc_offset)

            if args.mode == "baseline_xyz":
                # GPST date time e n u ...
                x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
            else:
                # GPST date time lat lon h ...
                x, y, z = float(parts[2]), float(parts[3]), float(parts[4])

            # Orientation unknown in RTK .pos -> identity quaternion
            out_lines.append(f"{ts:.6f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1\n")

    with open(args.out_tum, "w", encoding="utf-8") as f:
        f.writelines(out_lines)

    print(f"Wrote {len(out_lines)} poses to {args.out_tum}")


if __name__ == "__main__":
    main()
