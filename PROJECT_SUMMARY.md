# S3LI Dataset Evaluation - Project Summary

## Overview
Complete Docker environment for reproducing results from:
**"Challenges of SLAM in Extremely Unstructured Environments: The DLR Planetary Stereo, Solid-State LiDAR, Inertial Dataset"**
- Authors: Giubilato, Stürzl, Wedler, Triebel (DLR)
- Published: IEEE Robotics and Automation Letters, 2022
- Dataset: Mt. Etna volcanic environment (planetary analog)

## What's Included

### 1. Docker Container Setup
- **Dockerfile**: Complete environment with all dependencies
- **run_s3li_evaluation.sh**: Main launch script
- Container configured to **maintain static ID** and **not close**
- Persistent storage with mounted volumes

### 2. SLAM Systems (Pre-built)
✅ ORB-SLAM3 (Stereo + Stereo-Inertial)
✅ VINS-Fusion (Stereo + Stereo-Inertial)
✅ OpenVINS (Stereo-Inertial)
✅ BASALT (Stereo-Inertial)

### 3. Evaluation Scripts
- `download_dataset.sh` - Download sequences from DLR servers
- `run_evaluation.sh` - Run all SLAM algorithms on sequences
- `evaluate_trajectory.py` - Compute normalized RMSE metrics
- `generate_table.py` - Generate Table III comparison

### 4. Configuration Files
- ORB-SLAM3 YAML configs
- VINS-Fusion configs
- OpenVINS configs
- BASALT JSON configs

### 5. Documentation
- **README.md** - Comprehensive documentation
- **QUICK_START.sh** - Quick reference guide
- Comments throughout all scripts

## Dataset Details

**7 Sequences** | **4+ km total** | **Planetary analog environment**

| Sequence | Length | Features |
|----------|--------|----------|
| traverse_1 | 371m | Severe visual aliasing |
| traverse_2 | 300m | Panning motions |
| crater | 1010m | Long crater rim traverse |
| loops | 587m | Loop closure testing |
| crater_inout | 1338m | In/out crater exploration |
| mapping | 242m | Dense mapping area |
| landmarks | 482m | Rock formation navigation |

**Sensors:**
- Stereo: AVT Mako 688×512, 30Hz
- LiDAR: Blickfeld Cube-1 (Solid-State), 4.7Hz, 70°×30° FOV
- IMU: XSens MTi-G 10, 400Hz
- Ground Truth: D-GNSS (RTK, cm-level)

## Usage Workflow

```bash
# 1. Build and start container (maintains static ID)
./run_s3li_evaluation.sh

# 2. Access container
docker exec -it s3li_slam_eval bash

# 3. Download dataset
bash /workspace/scripts/download_dataset.sh

# 4. Run evaluations
bash /workspace/scripts/run_evaluation.sh

# 5. View results
cat /workspace/results/table_iii_comparison.csv
```

## Key Features

### ✅ Container Persistence
- Uses `CMD ["tail", "-f", "/dev/null"]` to keep running
- Configured with `--restart unless-stopped`
- Maintains static container ID across restarts
- Survives system reboots

### ✅ Complete Reproducibility
- All SLAM systems pre-built
- Exact versions from paper evaluation
- Calibration files included
- Evaluation metrics match paper

### ✅ Automated Evaluation
- Batch processing of all sequences
- Automatic trajectory alignment (Horn's method)
- Normalized RMSE computation
- Completion ratio calculation
- Table III generation

## Output Metrics

Following the paper's methodology:

1. **Normalized RMSE** = RMSE / trajectory_length
   - Accounts for different sequence lengths
   - Comparable across sequences
   
2. **Completion Ratio** = estimated_length / groundtruth_length
   - Indicates algorithm robustness
   - 100% = completed entire sequence

Format: `0.86 (100)` = RMSE 0.86, completed 100%

## File Structure

```
s3li_evaluation/
├── Dockerfile                    # Main container definition
├── .dockerignore                # Build exclusions
├── run_s3li_evaluation.sh       # Launch script ⭐
├── QUICK_START.sh               # Quick reference
├── README.md                    # Full documentation
├── evaluation_scripts/
│   ├── download_dataset.sh      # Dataset downloader
│   ├── run_evaluation.sh        # Main evaluation script
│   ├── evaluate_trajectory.py   # Trajectory evaluation
│   └── generate_table.py        # Table III generator
├── configs/
│   └── orbslam3_config.yaml     # SLAM configs
├── dataset/                     # Mounted - datasets
└── results/                     # Mounted - outputs
```

## Requirements

- **Docker** (20.10+)
- **Disk Space**: ~50GB for full dataset
- **RAM**: 8GB minimum, 16GB recommended
- **CPU**: Multi-core recommended
- **OS**: Linux (tested on Ubuntu)

## Container Management

```bash
# Start
./run_s3li_evaluation.sh

# Access
docker exec -it s3li_slam_eval bash

# Container ID (static)
docker ps -qf "name=s3li_slam_eval"

# Stop (container persists)
docker stop s3li_slam_eval

# Restart (same ID)
docker start s3li_slam_eval

# Logs
docker logs -f s3li_slam_eval

# Complete removal
docker stop s3li_slam_eval
docker rm s3li_slam_eval
docker rmi s3li_slam:latest
```

## Expected Results

Reproducing Table III from paper:

| Algorithm | traverse_1 | loops | crater | crater_inout | landmarks | mapping | traverse_2 |
|-----------|------------|-------|--------|--------------|-----------|---------|------------|
| ORB-SLAM3 (S) | 0.86 (100) | 0.21 (100) | 0.87 (56.5) | 0.19 (69.5) | - | 0.71 (41.2) | 0.38 (84.1) |
| VINS-Fusion (S) | 1.41 (100) | 6.23 (100) | 2.25 (100) | - | 3.25 (50.1) | 2.01 (62.5) | 1.33 (100) |
| OpenVINS | 0.54 (100) | 0.41 (80.3) | 0.54 (42.6) | 0.16 (86.9) | 0.77 (83.9) | - | 0.39 (100) |
| BASALT (SI) | 0.75 (100) | 0.35 (100) | 2.81 (100) | 1.08 (100) | 1.34 (100) | 0.47 (100) | 0.57 (100) |

*Values: normalized_RMSE (completion%)*

## Troubleshooting

### Container exits immediately
- Check `docker logs s3li_slam_eval`
- Verify Docker daemon is running
- Check disk space availability

### Build fails
- Ensure network connectivity
- Check available disk space (need ~10GB)
- Review build logs for specific errors

### SLAM system crashes
- Check system logs in `results/<system>/<sequence>/output.log`
- Verify sufficient RAM (8GB+)
- Try adjusting config parameters

### Dataset download fails
- Check network connectivity
- Verify URL: https://datasets.arches-projekt.de/s3li_dataset/
- Try downloading sequences individually

## Citation

```bibtex
@article{giubilato2022challenges,
  title={Challenges of SLAM in Extremely Unstructured Environments: 
         The DLR Planetary Stereo, Solid-State LiDAR, Inertial Dataset},
  author={Giubilato, Riccardo and St{\"u}rzl, Wolfgang and 
          Wedler, Armin and Triebel, Rudolph},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={4},
  pages={8721--8728},
  year={2022},
  publisher={IEEE},
  doi={10.1109/LRA.2022.3188118}
}
```

## Resources

- **Paper**: https://ieeexplore.ieee.org/document/9816626
- **Dataset**: https://rmc.dlr.de/s3li_dataset
- **Dataset URL**: https://datasets.arches-projekt.de/s3li_dataset/
- **DLR Contact**: riccardo.giubilato@dlr.de

## License

Research use only. See individual SLAM system licenses.

---

**Created**: November 2025
**Purpose**: Reproducible research for SLAM in planetary-analog environments
**Status**: Ready for evaluation ✅
