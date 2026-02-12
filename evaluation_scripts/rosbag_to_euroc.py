#!/usr/bin/env python3
"""Convert ROS1 stereo image topics in a rosbag to EuRoC-style dataset.

Outputs:
  <out_dir>/cam0/data/<timestamp>.png
  <out_dir>/cam1/data/<timestamp>.png
  <out_dir>/cam0/data.csv
  <out_dir>/cam1/data.csv
  <out_dir>/timestamps.txt
"""

import argparse
import os
import sys

import rosbag
from cv_bridge import CvBridge
import cv2


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def write_csv(path: str, rows):
    with open(path, "w", encoding="utf-8") as f:
        f.write("#timestamp [ns],filename\n")
        for ts_ns, filename in rows:
            f.write(f"{ts_ns},{filename}\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert rosbag stereo images to EuRoC format")
    parser.add_argument("--bag", required=True, help="Path to rosbag")
    parser.add_argument("--left", required=True, help="Left image topic (sensor_msgs/Image)")
    parser.add_argument("--right", required=True, help="Right image topic (sensor_msgs/Image)")
    parser.add_argument("--out", required=True, help="Output directory (EuRoC-style)")
    parser.add_argument("--max", type=int, default=0, help="Optional max frames per camera (0 = no limit)")
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        print(f"Bag not found: {args.bag}", file=sys.stderr)
        return 1

    cam0_data = os.path.join(args.out, "cam0", "data")
    cam1_data = os.path.join(args.out, "cam1", "data")
    ensure_dir(cam0_data)
    ensure_dir(cam1_data)

    bridge = CvBridge()

    left_rows = []
    right_rows = []
    left_count = 0
    right_count = 0

    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, _ in bag.read_messages(topics=[args.left, args.right]):
            ts_ns = msg.header.stamp.to_nsec()
            if topic == args.left:
                if args.max and left_count >= args.max:
                    continue
                img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                filename = f"{ts_ns}.png"
                cv2.imwrite(os.path.join(cam0_data, filename), img)
                left_rows.append((ts_ns, f"data/{filename}"))
                left_count += 1
            elif topic == args.right:
                if args.max and right_count >= args.max:
                    continue
                img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                filename = f"{ts_ns}.png"
                cv2.imwrite(os.path.join(cam1_data, filename), img)
                right_rows.append((ts_ns, f"data/{filename}"))
                right_count += 1

    left_rows.sort(key=lambda x: x[0])
    right_rows.sort(key=lambda x: x[0])

    write_csv(os.path.join(args.out, "cam0", "data.csv"), left_rows)
    write_csv(os.path.join(args.out, "cam1", "data.csv"), right_rows)

    with open(os.path.join(args.out, "timestamps.txt"), "w", encoding="utf-8") as f:
        for ts_ns, _ in left_rows:
            f.write(f"{ts_ns}\n")

    print(f"Left images:  {left_count}")
    print(f"Right images: {right_count}")
    print(f"Output dir:   {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
