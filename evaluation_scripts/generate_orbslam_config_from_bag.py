#!/usr/bin/env python3
"""
Generate an ORB-SLAM3 stereo YAML config from ROS1 camera_info topics.

Reads one message from left/right camera_info to extract intrinsics and baseline.
Writes a headless (Viewer: 0) ORB-SLAM3 config.
"""

import argparse
import os
import sys

import rosbag


def read_camera_info(bag_path: str, topic: str):
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            return msg
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate ORB-SLAM3 config from rosbag camera_info")
    parser.add_argument("--bag", required=True, help="Path to rosbag")
    parser.add_argument("--left-info", required=True, help="Left camera_info topic")
    parser.add_argument("--right-info", required=True, help="Right camera_info topic")
    parser.add_argument("--fps", type=float, default=30.0, help="Camera FPS (default: 30)")
    parser.add_argument("--out", required=True, help="Output YAML path")
    parser.add_argument("--viewer", type=int, default=0, help="Viewer on/off (0 = headless)")
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        print(f"Bag not found: {args.bag}", file=sys.stderr)
        return 1

    left = read_camera_info(args.bag, args.left_info)
    right = read_camera_info(args.bag, args.right_info)
    if left is None or right is None:
        print("Failed to read camera_info from bag. Check topic names.", file=sys.stderr)
        return 1

    # Intrinsics from left K matrix
    fx = left.K[0]
    fy = left.K[4]
    cx = left.K[2]
    cy = left.K[5]

    width = left.width
    height = left.height

    # Baseline from right P matrix: P[0,3] = -fx * baseline
    # P is 3x4 row-major, so index 3 is P[0,3]
    p03 = right.P[3]
    if fx == 0:
        print("Invalid fx from camera_info.", file=sys.stderr)
        return 1
    baseline = -p03 / fx
    bf = baseline * fx

    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    with open(args.out, "w", encoding="utf-8") as f:
        f.write("%YAML:1.0\n")
        f.write("---\n\n")
        f.write('Camera.type: "PinHole"\n')
        f.write(f"Camera.fx: {fx}\n")
        f.write(f"Camera.fy: {fy}\n")
        f.write(f"Camera.cx: {cx}\n")
        f.write(f"Camera.cy: {cy}\n\n")

        # Assume rectified images (no distortion)
        f.write("Camera.k1: 0.0\n")
        f.write("Camera.k2: 0.0\n")
        f.write("Camera.p1: 0.0\n")
        f.write("Camera.p2: 0.0\n")
        f.write("Camera.k3: 0.0\n\n")

        f.write(f"Camera.width: {width}\n")
        f.write(f"Camera.height: {height}\n")
        f.write(f"Camera.fps: {args.fps}\n")
        f.write(f"Camera.bf: {bf}\n")
        f.write("Camera.RGB: 1\n\n")

        # Depth thresholding
        f.write("ThDepth: 35.0\n")
        f.write("DepthMapFactor: 1.0\n\n")

        # ORB extractor parameters (defaults commonly used by ORB-SLAM3)
        f.write("ORBextractor.nFeatures: 2000\n")
        f.write("ORBextractor.scaleFactor: 1.2\n")
        f.write("ORBextractor.nLevels: 8\n")
        f.write("ORBextractor.iniThFAST: 20\n")
        f.write("ORBextractor.minThFAST: 5\n\n")

        # Viewer (0 = headless)
        f.write(f"Viewer: {args.viewer}\n")
        f.write("Viewer.KeyFrameSize: 0.05\n")
        f.write("Viewer.KeyFrameLineWidth: 1\n")
        f.write("Viewer.GraphLineWidth: 0.9\n")
        f.write("Viewer.PointSize: 2\n")
        f.write("Viewer.CameraSize: 0.08\n")
        f.write("Viewer.CameraLineWidth: 3\n")
        f.write("Viewer.ViewpointX: 0\n")
        f.write("Viewer.ViewpointY: -0.7\n")
        f.write("Viewer.ViewpointZ: -1.8\n")
        f.write("Viewer.ViewpointF: 500\n")

    print(f"ORB-SLAM3 config written to: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
