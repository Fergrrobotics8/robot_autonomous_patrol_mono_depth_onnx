# Fergr Robot (based on Nomeer robot) - Autonomous Navigation with Monocular Depth Vision

A complete autonomous robotic system built on ROS 2 Humble with integrated SLAM and autonomous navigation:
- SLAM: Real-time simultaneous localization and mapping
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

**IMPORTANT: Follow the order carefully!**

1. Terminal 1 - Start robot simulator:
```bash
ros2 launch robot_description robot.launch.py
```
   - Gazebo and RViz will open automatically
   - **IMPORTANT:** Press the PLAY button in Gazebo to start the simulation

2. Terminal 2 - Start SLAM:
```bash
ros2 run autonomous_patrol slam
```

# IF YOU WANT TO RECORD WAYPOINTS (3 to 6)

3. Terminal 3 - Teleoperate robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

4. Terminal 4 - Start waypoint recorder:
```bash
ros2 launch autonomous_patrol record_waypoints.launch.py
```

5. Navigate the robot using keyboard (w/x forward/backward, a/d turn)
6. Press Ctrl+C in Terminal 4 to save waypoints

Waypoints saved to: `src/nomeer_robot_ros2/src/autonomous_patrol/data/waypoints.yaml`



## Part B: Monocular Depth Vision with AI

**Advanced Configuration**

### Depth Inference and depth metric 

For testing depth vision independently:

1. Terminal 1 - Start depth inference:
```bash
ros2 launch mono_depth_onnx inference.launch.py
```

You can view the depth coloured in the rviz visualizer and see if there is an obstacle in the terminal (CLEAR=NO OBSTACLE)

The viewer displays:
- Colored depth visualization (red=close, blue=far)
- Depth metrics in real-time

### Depth Configuration

Edit configuration file for advanced settings:
```bash
nano src/nomeer_robot_ros2/src/mono_depth_onnx/config/mono_depth_config.yaml
```


### Playing Back Trajectory

**No need to close anything!** Simply:


1. In Terminal 4 (or a new Terminal 5), start the waypoint follower:
```bash
ros2 launch autonomous_patrol follow_waypoints.launch.py
```

3. The robot will automatically follow the saved trajectory. Watch the visualization in RViz that is already open

**Metrics** are automatically saved to: `src/nomeer_robot_ros2/src/autonomous_patrol/results/metrics.json`


Key parameters:
- `roi_x_start/roi_x_end`: Region of interest (horizontal, 0-1)
- `roi_y_start/roi_y_end`: Region of interest (vertical, 0-1)
- `obstacle_threshold`: Detection threshold (0=far, 1=close)
- `outlier_percentile_low/high`: Outlier filtering (5/95 = remove extreme 5%)

### Published Topics

The depth pipeline publishes:
- `/camera/depth_estimated` - Raw depth map (mono16, 0-65535)
- `/camera/depth_colored` - Colored visualization (BGR8)
- `/depth_metric/median_frontal_depth` - Median depth in ROI (0-1)
- `/depth_metric/min_frontal_depth` - Minimum depth in ROI (0-1)
- `/depth_metric/avg_frontal_depth` - Average depth in ROI (0-1)
- `/depth_metric/obstacle_detected` - Boolean flag (1=obstacle, 0=clear)

## Integration: SLAM + Autonomous Navigation + Depth Vision with Safety

**Complete system with obstacle detection and emergency stop**

### Launch Sequence (Follow order and timings!)

#### Step 1: Robot Simulator (Terminal 1)
```bash
ros2 launch robot_description robot.launch.py
```
**Expected output:**
- Gazebo window opens with simulated robot
- RViz window opens with visualization
- ⚠️ **IMPORTANT:** Click the PLAY button (▶) in Gazebo to start physics simulation
- **Duration:** 5-10 seconds to stabilize

#### Step 2: SLAM (Terminal 2) - Wait 5 seconds after step 1
```bash
# Start SLAM for real-time mapping
ros2 run autonomous_patrol slam
```
**Expected output:**
- SLAM output: "Starting SLAM" message
- RViz shows building map in green points
- **Duration:** 5-10 seconds to initialize

#### Step 3: Depth Inference (Terminal 3) - Wait 5 seconds after step 2
```bash
# Start depth inference with obstacle detection (MUST be running before follow_waypoints)
ros2 launch mono_depth_onnx inference.launch.py
```
**Expected output:**
- Depth nodes initialize model (MiDaS loads ~35MB)
- Terminal shows frame processing messages
- Depth colored visualization appears in RViz
- Obstacle detection ready
- **Duration:** 10-15 seconds for model load and initialization

#### Step 4: Autonomous Navigation (Terminal 4) - Wait 5 seconds after step 3
```bash
# Start waypoint follower with integrated safety (depth inference MUST be running)
ros2 launch autonomous_patrol follow_waypoints.launch.py
```
**Expected output:**
- "Starting autonomous navigation" message
- Robot begins moving in Gazebo
- RViz shows waypoints in red and current heading
- Terminal logs each waypoint: "Waypoint N reached"
- If obstacle nearby: `🚨 OBSTACLE DETECTED - EMERGENCY STOP!`

### How It Works

1. **Robot moves along recorded waypoints** - Following trajectory from `waypoints.yaml`
2. **Depth sensor runs continuously** - Analyzing center region (15%-85% x 15%-65%)
3. **If obstacle detected** (median depth > 0.6 = RED in visualization):
   - ⛔ **IMMEDIATE STOP** - Robot velocity set to zero
   - 🚨 Alert logged with counter: `OBSTACLE DETECTED #{count} - EMERGENCY STOP ACTIVATED!`
   - Recorded in `metrics.json` for post-analysis
4. **Once obstacle cleared**, autonomous navigation automatically resumes

### Safety Configuration

Adjust detection sensitivity in config file:
```yaml
# ~/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/config/mono_depth_config.yaml
depth_metric:
  # Obstacle threshold: median depth > this value triggers STOP
  # Range: 0.0 (far, blue) to 1.0 (close, red)
  obstacle_threshold: 0.6  # Default: detects red objects
  
  # Region of Interest (center of image, avoiding sides/floor)
  roi_x_start: 0.15        # 15% from left
  roi_x_end: 0.85          # 85% from left
  roi_y_start: 0.15        # 15% from top
  roi_y_end: 0.65          # 65% from top (avoids floor)
```

### Metrics & Results

After execution completes, check:
- **Waypoint metrics:** `src/nomeer_robot_ros2/src/autonomous_patrol/results/metrics.json`
  - `waypoints_completed` - How many waypoints were reached
  - `total_distance` - Distance traveled
  - `obstacle_stops` - Number of emergency stops triggered ⚠️
- **Real-time monitoring:** Watch `/depth_metric/obstacle_detected` topic during execution
- **Depth visualization:** Green box in RViz shows analyzed ROI region

## SLAM: Simultaneous Localization and Mapping

Real-time SLAM for robot mapping and localization:

```bash
ros2 run autonomous_patrol slam
```

This runs during autonomy:
- Builds real-time environment map
- Localizes the robot within the map
- Provides accurate odometry for autonomous navigation

Published topics:
- `/map` - Generated map
- `/tf` - Transformation tree (robot pose)
- `/scan` - Laser/sensor data

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
