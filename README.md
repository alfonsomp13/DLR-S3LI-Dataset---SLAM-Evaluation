# DLR S3LI Dataset - SLAM Evaluation Docker Environment

This Docker environment reproduces the results from the paper:
**"Challenges of SLAM in Extremely Unstructured Environments: The DLR Planetary Stereo, Solid-State LiDAR, Inertial Dataset"**
by Giubilato et al., IEEE RA-L 2022

## Dataset Information

The S3LI dataset was recorded on Mt. Etna, Sicily, a planetary analogous environment with:
- **7 sequences** covering over 4 kilometers
- **Stereo camera** (AVT Mako, 688×512 pixels, 30 Hz)
- **Solid-State LiDAR** (Blickfeld Cube-1, 70°H × 30°V FOV, 4.7 Hz)
- **IMU** (XSens MTi-G 10, 400 Hz)
- **D-GNSS ground truth** (centimeter-level accuracy)

Dataset URL: https://datasets.arches-projekt.de/s3li_dataset/

## Quick Start

### 1. Build and Run Container

```bash
chmod +x run_s3li_evaluation.sh
./run_s3li_evaluation.sh
```

This will:
- Build the Docker image with all SLAM systems
- Start a persistent container (won't close, maintains static ID)
- Mount directories for dataset and results
- Enable X11 forwarding for viewer-based SLAM runs

If you plan to use the ORB-SLAM3 viewer, allow local X11 access on the host:

```bash
xhost +local:root
```

If you are on a remote host, ensure `DISPLAY` is set correctly before running the container:

```bash
echo $DISPLAY
```

For headless/limited OpenGL environments, force software rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
```

If you encounter `free(): invalid pointer` crashes during optimization, rebuild the image. The Dockerfile pins Eigen alignment settings for stability in g2o:

```
-DEIGEN_DONT_ALIGN_STATICALLY -DEIGEN_DONT_VECTORIZE
```

### 2. Access Container

```bash
docker exec -it s3li_slam_eval bash
```

### 3. Download Dataset

Inside the container:
```bash
bash /workspace/scripts/download_dataset.sh
```

You can select individual sequences or download all (~50GB total).

### 4. Run Evaluations

```bash
bash /workspace/scripts/run_evaluation.sh
```

This will:
- Run all SLAM algorithms on selected sequences
- Compute normalized RMSE and completion ratios
- Generate Table III comparison from the paper

## Build Notes (ORB-SLAM3 + Pangolin)

The Docker image now builds Pangolin from source and relaxes strict warning flags so ORB-SLAM3 compiles reliably with OpenCV 4.2 on Ubuntu 20.04.

If you need to rebuild ORB-SLAM3 **inside the container** (for example after changing configs), run:

```bash
cd /workspace/ORB_SLAM3
./build.sh
```

If the build fails with `PangolinConfig.cmake not found`, it means Pangolin was not installed correctly in the image. Rebuild the Docker image from the host to restore Pangolin and ORB-SLAM3:

```bash
docker build -t s3li_slam:latest .
```

## SLAM Systems Included

The following SLAM systems are pre-built in the container:

1. **ORB-SLAM3** (Stereo & Stereo-Inertial)
2. **VINS-Fusion** (Stereo & Stereo-Inertial)  
3. **OpenVINS** (Stereo-Inertial)
4. **BASALT** (Stereo-Inertial)

## Sequences

| Sequence | Length | Description |
|----------|--------|-------------|
| s3li_traverse_1 | 371 m | Ash slope traverse, severe visual aliasing |
| s3li_traverse_2 | 300 m | Short traverse with panning motions |
| s3li_crater | 1010 m | Long traverse around crater rim |
| s3li_loops | 587 m | Loop closure opportunities on rocky ridges |
| s3li_crater_inout | 1338 m | Long traverse in/out of crater |
| s3li_mapping | 242 m | Dense mapping area |
| s3li_landmarks | 482 m | Traverses between rock formations |

## Directory Structure

```
/workspace/
├── dataset/              # Dataset storage
│   ├── s3li_traverse_1/
│   ├── s3li_traverse_2/
│   └── ...
├── results/              # Evaluation results
│   ├── orbslam3_stereo/
│   ├── vins_fusion_stereo/
│   └── ...
├── scripts/              # Evaluation scripts
│   ├── download_dataset.sh
│   ├── run_evaluation.sh
│   ├── evaluate_trajectory.py
│   └── generate_table.py
├── configs/              # SLAM configuration files
├── ORB_SLAM3/           # ORB-SLAM3 source
├── catkin_ws/           # ROS workspace (VINS, OpenVINS)
└── basalt/              # BASALT source
```

## Output Files

After running evaluations, results are saved to `/workspace/results/`:

- `evaluation_<sequence>.csv` - Per-sequence results
- `table_iii_comparison.csv` - Main comparison table
- `table_iii_comparison_detailed.csv` - Detailed statistics
- `<system>/<sequence>/trajectory.txt` - Estimated trajectories
- `<system>/<sequence>/output.log` - System logs

## Reproducing Table III

To reproduce Table III from the paper:

```bash
# Inside container
bash /workspace/scripts/run_evaluation.sh

# Select option: Run all sequences (option 8)
# Wait for completion (may take several hours)

# Results will be in:
cat /workspace/results/table_iii_comparison.csv
```

## Manual Evaluation

To run individual SLAM systems manually:

### 0) Prepare ORB-SLAM3 inputs (required)

#### A) Convert rosbag -> EuRoC (stereo sync)
```bash
python3 /workspace/scripts/rosbag_to_euroc.py \
  --bag /workspace/dataset/HiDrive/Bagfiles/s3li_traverse_1.bag \
  --left /stereo/left/image_rect \
  --right /stereo/right/image_rect \
  --out /workspace/dataset/converted/s3li_traverse_1
```

#### B) Generate ORB-SLAM3 config from camera_info (headless-ready)
```bash
python3 /workspace/scripts/generate_orbslam_config_from_bag.py \
  --bag /workspace/dataset/HiDrive/Bagfiles/s3li_traverse_1.bag \
  --left-info /stereo/left/camera_info \
  --right-info /stereo/right/camera_info \
  --fps 30 \
  --out /workspace/configs/orbslam_config_headless.yaml \
  --viewer 0
```

### ORB-SLAM3 (Stereo)
```bash
bash /workspace/scripts/run_orbslam3_stereo.sh \
    --seq-dir /workspace/dataset/converted/s3li_traverse_1 \
    --mode headless \
    --output-dir /workspace/results/orbslam3_s3li_traverse_1
```

If you want the GUI viewer, run inside the container **without** `xvfb-run` and set `Viewer: 1` in the config. Note that some setups crash during viewer teardown; results are still written to disk.

For GUI mode with host X11:
```bash
bash /workspace/scripts/run_orbslam3_stereo.sh \
    --seq-dir /workspace/dataset/converted/s3li_traverse_1 \
    --mode gui \
    --output-dir /workspace/results/orbslam3_s3li_traverse_1
```

`run_s3li_evaluation.sh` now auto-configures:
- `xhost +SI:localuser:root` (best effort)
- host `.Xauthority` mount into the container (when available)

This prevents the common runtime error:
`Authorization required, but no authorization protocol specified`

### Evaluate ORB-SLAM3 vs Ground Truth

After ORB-SLAM3 finishes, evaluate trajectory error with:

For `s3li_traverse_1`, ground truth is located at:
- `/workspace/dataset/HiDrive/GT/s3li_traverse_1/baseline_xyz.pos`
- `/workspace/dataset/HiDrive/GT/s3li_traverse_1/global_lle.pos`

Convert `baseline_xyz.pos` to TUM first:

```bash
python3 /workspace/scripts/pos_to_tum.py \
  --in-pos /workspace/dataset/HiDrive/GT/s3li_traverse_1/baseline_xyz.pos \
  --out-tum /workspace/results/s3li_traverse_1_gt_tum.txt \
  --mode baseline_xyz
```

```bash
bash /workspace/scripts/evaluate_orbslam_vs_gt.sh \
  --gt /workspace/results/s3li_traverse_1_gt_tum.txt \
  --est /workspace/results/orbslam3_s3li_traverse_1/CameraTrajectory.txt \
  --format tum \
  --t-max-diff 0.05 \
  --out-root /workspace/results
```

Note: `evaluate_orbslam_vs_gt.sh` now auto-normalizes TUM timestamps from nanoseconds to seconds (common in ORB-SLAM3 outputs) before running `evo`.

This generates a timestamped folder in `/workspace/results/` with:
- `ape.zip`, `rpe.zip` and `summary.json`
- `ape_plot.pdf`, `rpe_plot.pdf`, `traj_overlay.pdf`
- console logs for reproducibility

If `evo` fails with `ModuleNotFoundError: No module named 'packaging'` or NumPy/SciPy compatibility warnings, rebuild the Docker image. The Dockerfile now pins compatible Python 3.8 scientific packages and installs `packaging`.

### VINS-Fusion
```bash
cd /workspace/catkin_ws
source devel/setup.bash
roslaunch vins vins_rviz.launch
# In another terminal: play dataset
```

### OpenVINS
```bash
cd /workspace/catkin_ws
source devel/setup.bash
roslaunch ov_msckf pgeneva_ros_eth.launch
```

### BASALT
```bash
cd /workspace/basalt/build
./basalt_vio \
    --dataset-path /workspace/dataset/s3li_traverse_1 \
    --config-path /workspace/configs/basalt_config.json \
    --save-trajectory
```

## Container Management

### Stop Container
```bash
docker stop s3li_slam_eval
```

### Restart Container
```bash
docker start s3li_slam_eval
docker exec -it s3li_slam_eval bash
```

### View Container Logs
```bash
docker logs -f s3li_slam_eval
```

### Get Container ID
```bash
docker ps -qf "name=s3li_slam_eval"
```

The container is configured with `--restart unless-stopped`, ensuring it maintains a static ID and survives system reboots.

### Remove Container
```bash
docker stop s3li_slam_eval
docker rm s3li_slam_eval
```

### Remove Image
```bash
docker rmi s3li_slam:latest
```

## Configuration

Configuration files for each SLAM system are in `/workspace/configs/`. 
Adjust these to modify algorithm parameters:

- `orbslam3_config.yaml` - ORB-SLAM3 parameters
- `vins_config.yaml` - VINS-Fusion parameters
- `openvins_config.yaml` - OpenVINS parameters  
- `basalt_config.json` - BASALT parameters

## Evaluation Metrics

As described in the paper, the evaluation computes:

1. **Normalized RMSE**: RMSE / trajectory_length
   - Accounts for different trajectory lengths
   - Lower is better

2. **Completion Ratio**: estimated_length / groundtruth_length
   - Indicates how much of the sequence was successfully processed
   - Higher is better (100% = complete sequence)

Results are formatted as: `RMSE (Completion%)`

## Key Findings from Paper

The paper demonstrates that:
- Severe visual aliasing challenges place recognition
- Lack of structural features limits LiDAR-only SLAM
- Stereo-inertial systems show best overall performance
- ORB-SLAM3 struggles with feature-poor environments
- OpenVINS and BASALT are more robust to aliasing

## Troubleshooting

### Dataset download fails
- Check network connectivity
- Verify dataset URL is accessible
- Try downloading individual sequences

### SLAM system crashes
- Check system logs in `results/<system>/<sequence>/output.log`
- Verify sufficient memory (8GB+ recommended)
- Adjust parameters in config files

### ROS issues
```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash
```

### Build errors
Container includes pre-built systems, but to rebuild:
```bash
cd /workspace/ORB_SLAM3
./build.sh

cd /workspace/catkin_ws
catkin_make
```

## Citation

If you use this dataset or code, please cite:

```bibtex
@article{giubilato2022challenges,
  title={Challenges of SLAM in Extremely Unstructured Environments: The DLR Planetary Stereo, Solid-State LiDAR, Inertial Dataset},
  author={Giubilato, Riccardo and St{\"u}rzl, Wolfgang and Wedler, Armin and Triebel, Rudolph},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={4},
  pages={8721--8728},
  year={2022},
  publisher={IEEE}
}
```

## Contact

For dataset issues: https://rmc.dlr.de/s3li_dataset
For paper questions: riccardo.giubilato@dlr.de

## License

Dataset and code are provided for research purposes. 
See individual SLAM system licenses for their respective terms.
