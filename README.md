# Nomeer Robot ROS 2 Workspace

## 🎯 Quick Overview

This workspace contains a complete autonomous robotic system with:
- **Part A**: Waypoint-based autonomy with trajectory recording
- **Part B**: Monocular depth perception with ONNX-accelerated AI

## ⚡ Quick Start

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_patrol mono_depth_onnx
source install/setup.bash
ros2 launch mono_depth_onnx full_pipeline.launch.py
```

## 📚 Documentation

- **[EXECUTIVE_SUMMARY.md](EXECUTIVE_SUMMARY.md)** - Project overview and status
- **[DOCUMENTATION_INDEX.md](DOCUMENTATION_INDEX.md)** - Navigation guide for all docs
- **[src/nomeer_robot_ros2/README.md](src/nomeer_robot_ros2/README.md)** - Full system documentation

## 📦 Packages

- `robot_description` - Base robot model
- `autonomous_patrol` - Part A: Autonomy system
- `mono_depth_onnx` - Part B: Depth estimation

## ✅ Verification

```bash
bash verify_installation.sh
```

---

**Status**: ✅ Production Ready | **Date**: 2026-02-12

