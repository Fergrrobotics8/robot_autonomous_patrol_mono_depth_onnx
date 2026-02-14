# Nomeer Robot - Autonomous Navigation with Monocular Depth Vision

A complete autonomous robotic system built on ROS 2 Humble with two integrated capabilities:
- Part A: Waypoint-based autonomous navigation with trajectory recording
- Part B: Real-time monocular depth estimation using ONNX-accelerated AI (MiDaS)

## Quick Start

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# Initial setup (one time)
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
pip3 install 'numpy<2' opencv-python onnxruntime PyYAML scipy torch timm onnx onnxscript

# Build packages
colcon build --packages-select autonomous_patrol mono_depth_onnx robot_description
source install/setup.bash

# Download AI model
cd src/nomeer_robot_ros2/src/mono_depth_onnx
python3 scripts/download_midas_model.py
cd ~/ros2_ws
```

## Setup Instructions

### Initial Configuration (Once)

1. Source ROS 2 Humble:
```bash
source /opt/ros/humble/setup.bash
```

2. Install system dependencies:
```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y ros-humble-teleop-twist-keyboard
sudo apt install -y ros-humble-gazebo-ros
```

3. Install Python dependencies:
```bash
pip3 install 'numpy<2' opencv-python onnxruntime PyYAML scipy torch timm onnx onnxscript
```

4. Build all packages:
```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_patrol mono_depth_onnx robot_description
source install/setup.bash
```

5. Download AI model:
```bash
cd src/nomeer_robot_ros2/src/mono_depth_onnx
python3 scripts/download_midas_model.py
```

### Every Session

In each new terminal:
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Part A: Autonomous Waypoint Navigation

### Recording a Trajectory

1. Terminal 1 - Start robot simulator:
```bash
ros2 launch robot_description robot.launch.py
```

2. Terminal 2 - Start waypoint recorder:
```bash
ros2 launch autonomous_patrol record_waypoints.launch.py
```

3. Terminal 3 - Teleoperate robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

4. Navigate the robot using keyboard (w/x forward/backward, a/d turn)
5. Press Ctrl+C in Terminal 2 to save waypoints

Waypoints saved to: `src/nomeer_robot_ros2/src/autonomous_patrol/data/waypoints.yaml`

### Playing Back Trajectory

1. Terminal 1 - Start robot simulator:
```bash
ros2 launch robot_description robot.launch.py
```

2. Terminal 2 - Start waypoint follower:
```bash
ros2 launch autonomous_patrol follow_waypoints.launch.py
```

3. (Optional) Terminal 3 - Visualize in RViz:
```bash
ros2 launch autonomous_patrol visualize_waypoints.launch.py
```

Metrics saved to: `src/nomeer_robot_ros2/src/autonomous_patrol/results/metrics.json`

## Part B: Monocular Depth Vision with AI

### Quick Start - Full Pipeline

```bash
ros2 launch mono_depth_onnx full_pipeline.launch.py
```

This runs the complete pipeline with test images:
- Loads RGB images
- Executes MiDaS depth inference
- Calculates depth metrics
- Displays colored depth visualization

### With Live Webcam

Edit configuration:
```bash
nano src/nomeer_robot_ros2/src/mono_depth_onnx/config/mono_depth_config.yaml
```

Change to:
```yaml
image_source:
  source_type: "webcam"
  source_path: "0"
```

Then rebuild and launch:
```bash
colcon build --packages-select mono_depth_onnx
ros2 launch mono_depth_onnx full_pipeline.launch.py
```

### With Video File

Place video in the data directory:
```bash
cp your_video.mp4 src/nomeer_robot_ros2/src/mono_depth_onnx/data/video.mp4
```

Edit configuration:
```bash
nano src/nomeer_robot_ros2/src/mono_depth_onnx/config/mono_depth_config.yaml
```

Change to:
```yaml
image_source:
  source_type: "video"
  source_path: "data/video.mp4"
```

Then rebuild and launch:
```bash
colcon build --packages-select mono_depth_onnx
ros2 launch mono_depth_onnx full_pipeline.launch.py
```

### Inference Only (Subscribe to External RGB Source)

If you want to subscribe to your own RGB image publisher:
```bash
ros2 run mono_depth_onnx depth_inference_node.py
```

This node subscribes to `/rgb_image` and publishes:
- `/camera/depth_estimated` - Raw depth map (mono16)
- `/camera/depth_colored` - Colored depth visualization

## Integration: Autonomous Navigation + Vision + Safety

Run all three components together for autonomous navigation with depth-based obstacle detection:

Terminal 1 - Autonomy:
```bash
ros2 launch autonomous_patrol follow_waypoints.launch.py
```

Terminal 2 - Vision/Depth:
```bash
ros2 run mono_depth_onnx depth_inference_node.py
```

Terminal 3 - Safety (emergency stop if obstacle < 0.1m):
```bash
ros2 run mono_depth_onnx autonomous_depth_safety_node.py
```

## Important Files

```
src/nomeer_robot_ros2/
├── src/
│   ├── autonomous_patrol/
│   │   ├── data/waypoints.yaml              # Recorded trajectories
│   │   ├── results/metrics.json             # Execution metrics
│   │   └── config/autonomous_patrol_config.yaml
│   ├── mono_depth_onnx/
│   │   ├── models/midas_v21_small.onnx      # AI model
│   │   └── config/mono_depth_config.yaml
│   └── robot_description/
├── README.md
└── src/nomeer_robot_ros2/README.md          # Original project README
```

## Available ROS 2 Topics

### Autonomy (Part A)
- `/cmd_vel` - Robot velocity commands
- `/odom` - Robot odometry
- `/waypoint_follower/current_waypoint` - Current waypoint index
- `/waypoint_follower/status` - Status string

### Vision/Depth (Part B)
- `/rgb_image` - Input RGB image (for external sources)
- `/camera/depth_estimated` - Depth map (mono16, 0-65535)
- `/camera/depth_colored` - Colored depth visualization
- `/depth_metric/min_frontal_depth` - Minimum frontal depth (0-1)
- `/depth_metric/mean_depth` - Mean depth (0-1)
- `/depth_metric/obstacle_detected` - Boolean: obstacle present

## Build Commands

Build all packages:
```bash
colcon build --packages-select autonomous_patrol mono_depth_onnx robot_description
```

Build individual package:
```bash
colcon build --packages-select autonomous_patrol
colcon build --packages-select mono_depth_onnx
colcon build --packages-select robot_description
```

Clean build:
```bash
colcon clean all
colcon build --packages-select autonomous_patrol mono_depth_onnx robot_description
source install/setup.bash
```

## Verification

```bash
bash verify_installation.sh
```

Expected output: 38/38 CHECKS PASSED

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "Package not found" | `colcon build && source install/setup.bash` |
| "numpy: _ARRAY_API error" | `pip3 install 'numpy<2'` |
| "ONNX model not found" | `cd src/nomeer_robot_ros2/src/mono_depth_onnx && python3 scripts/download_midas_model.py` |
| "Gazebo won't open" | `sudo apt install -y ros-humble-gazebo-ros` |
| "Can't see topics" | Ensure `source install/setup.bash` in all terminals |
| "Permission denied" | `chmod +x src/nomeer_robot_ros2/src/*/scripts/*.py` |

## Project Structure

```
ros2_ws/
├── README.md              # This file
├── SETUP_GUIDE.md         # Detailed setup documentation
├── QUICK_START.md         # Quick reference in Spanish
├── src/
│   └── nomeer_robot_ros2/
│       ├── README.md      # Original project readme
│       └── src/
│           ├── autonomous_patrol/    # Part A: Waypoint navigation
│           ├── mono_depth_onnx/      # Part B: Depth vision
│           └── robot_description/    # Robot model
├── build/        # Build artifacts
├── install/      # Installed packages
└── log/          # Build logs
```

## System Requirements

- ROS 2 Humble
- Python 3.10+
- Ubuntu 22.04 or similar
- ONNX Runtime (installed via pip)
- OpenCV Python

Optional:
- NVIDIA CUDA for GPU acceleration

## Documentation

- `SETUP_GUIDE.md` - Comprehensive setup and usage guide
- `QUICK_START.md` - Quick reference (Spanish)
- `src/nomeer_robot_ros2/README.md` - Original project documentation

## License

See original project documentation.

---

**Status**: Production Ready  
**Date**: February 14, 2026
