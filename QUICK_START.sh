#!/bin/bash
# QUICK_START.sh - Quick reference for common commands

cat << 'EOF'
╔═══════════════════════════════════════════════════════════════╗
║                DLR S3LI Dataset Quick Start                   ║
║    Reproducing "Challenges of SLAM in Extremely              ║
║         Unstructured Environments" (IEEE RA-L 2022)          ║
╚═══════════════════════════════════════════════════════════════╝

📦 STEP 1: Build and Start Container
────────────────────────────────────────────────────────────────
./run_s3li_evaluation.sh

Container will:
  ✓ Build with all SLAM systems (ORB-SLAM3, VINS, OpenVINS, BASALT)
  ✓ Start with persistent ID (won't close)
  ✓ Mount dataset/ and results/ directories

📥 STEP 2: Download Dataset  
────────────────────────────────────────────────────────────────
docker exec -it s3li_slam_eval bash
# Inside container:
bash /workspace/scripts/download_dataset.sh

Sequences available (total ~50GB):
  1. s3li_traverse_1     - 371m ash slope
  2. s3li_traverse_2     - 300m short traverse  
  3. s3li_crater         - 1010m crater rim
  4. s3li_loops          - 587m loop closure test
  5. s3li_crater_inout   - 1338m long traverse
  6. s3li_mapping        - 242m dense mapping
  7. s3li_landmarks      - 482m rock formations

🚀 STEP 3: Run Evaluations
────────────────────────────────────────────────────────────────
# Inside container:
bash /workspace/scripts/run_evaluation.sh

This will:
  ✓ Run all 6 SLAM algorithms
  ✓ Compute normalized RMSE metrics
  ✓ Generate Table III from paper

📊 STEP 4: View Results
────────────────────────────────────────────────────────────────
# From host machine:
cat results/table_iii_comparison.csv

# Or inside container:
cat /workspace/results/table_iii_comparison.csv

Results format: RMSE_normalized (Completion%)
  Example: 0.86 (100) = RMSE 0.86, completed 100% of sequence

═══════════════════════════════════════════════════════════════

🔧 USEFUL COMMANDS

Access container:
  docker exec -it s3li_slam_eval bash

Check container status:
  docker ps -f name=s3li_slam_eval

View container ID:
  docker ps -qf name=s3li_slam_eval

Stop container:
  docker stop s3li_slam_eval

Restart container:
  docker start s3li_slam_eval

View logs:
  docker logs -f s3li_slam_eval

Remove container:
  docker stop s3li_slam_eval && docker rm s3li_slam_eval

═══════════════════════════════════════════════════════════════

📂 DIRECTORY STRUCTURE

Host machine:
  ./dataset/          - Downloaded sequences
  ./results/          - Evaluation outputs
  ./configs/          - SLAM configurations
  ./evaluation_scripts/ - Python/bash scripts

Inside container:
  /workspace/dataset/    - Dataset storage
  /workspace/results/    - Results output
  /workspace/ORB_SLAM3/  - ORB-SLAM3 system
  /workspace/catkin_ws/  - ROS workspace (VINS, OpenVINS)
  /workspace/basalt/     - BASALT system

═══════════════════════════════════════════════════════════════

⚙️ MANUAL SLAM RUNS

ORB-SLAM3 (Stereo):
  cd /workspace/ORB_SLAM3
  ./Examples/Stereo/stereo_euroc \\
    Vocabulary/ORBvoc.txt \\
    /workspace/configs/orbslam3_config.yaml \\
    /workspace/dataset/s3li_traverse_1 \\
    timestamps.txt

OpenVINS:
  source /opt/ros/noetic/setup.bash
  source /workspace/catkin_ws/devel/setup.bash
  roslaunch ov_msckf pgeneva_ros_eth.launch

BASALT:
  cd /workspace/basalt/build
  ./basalt_vio --dataset-path /workspace/dataset/s3li_traverse_1

═══════════════════════════════════════════════════════════════

📖 DATASET INFO

Paper: https://ieeexplore.ieee.org/document/9816626
Dataset: https://datasets.arches-projekt.de/s3li_dataset/
Citation: Giubilato et al., IEEE RA-L 2022

Key Challenges:
  ✗ Severe visual aliasing (repetitive terrain)
  ✗ Lack of distinct structural features
  ✗ Limited LiDAR Field-of-View (70°H × 30°V)
  ✗ Harsh lighting conditions
  ✗ Unstructured planetary-like environment

═══════════════════════════════════════════════════════════════

💡 TIPS

1. Container maintains static ID - survives reboots
2. Download sequences individually to save space
3. Results saved to both host and container
4. Full evaluation takes several hours
5. Requires ~8GB RAM minimum
6. Check logs if system crashes

═══════════════════════════════════════════════════════════════

For detailed documentation: cat README.md

EOF
