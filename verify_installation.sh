#!/bin/bash

# FINAL VERIFICATION SCRIPT
# This script validates that all project components are correctly installed and compiled

echo "================================"
echo "🔍 NOMEER ROBOT: FINAL VERIFICATION"
echo "================================"
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Counters
PASS=0
FAIL=0

# Function to print results
check_result() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ $1${NC}"
        ((PASS++))
    else
        echo -e "${RED}❌ $1${NC}"
        ((FAIL++))
    fi
}

echo "📦 CHECKING PACKAGE COMPILATION"
echo "================================"

# Check autonomous_patrol
if [ -d "/home/ferradar/ros2_ws/install/autonomous_patrol" ]; then
    echo -e "${GREEN}✅ autonomous_patrol package compiled${NC}"
    ((PASS++))
else
    echo -e "${RED}❌ autonomous_patrol package NOT compiled${NC}"
    ((FAIL++))
fi

# Check mono_depth_onnx
if [ -d "/home/ferradar/ros2_ws/install/mono_depth_onnx" ]; then
    echo -e "${GREEN}✅ mono_depth_onnx package compiled${NC}"
    ((PASS++))
else
    echo -e "${RED}❌ mono_depth_onnx package NOT compiled${NC}"
    ((FAIL++))
fi

echo ""
echo "📁 CHECKING DIRECTORY STRUCTURE"
echo "================================"

# Part A files
[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/README.md" ]
check_result "autonomous_patrol/README.md"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/autonomous_patrol/record_waypoints_node.py" ]
check_result "record_waypoints_node.py"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/autonomous_patrol/follow_waypoints_node.py" ]
check_result "follow_waypoints_node.py"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/autonomous_patrol/visualizer_node.py" ]
check_result "visualizer_node.py"

# Part B files
[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/README.md" ]
check_result "mono_depth_onnx/README.md"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/mono_depth_onnx/image_source_node.py" ]
check_result "image_source_node.py"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/mono_depth_onnx/depth_inference_node.py" ]
check_result "depth_inference_node.py"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/mono_depth_onnx/depth_metric_node.py" ]
check_result "depth_metric_node.py"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/mono_depth_onnx/autonomous_depth_safety_node.py" ]
check_result "autonomous_depth_safety_node.py"

# Scripts
[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/scripts/download_midas_model.py" ]
check_result "download_midas_model.py"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/scripts/generate_test_images.py" ]
check_result "generate_test_images.py"

echo ""
echo "📚 CHECKING DOCUMENTATION"
echo "================================"

[ -f "/home/ferradar/ros2_ws/EXECUTIVE_SUMMARY.md" ]
check_result "EXECUTIVE_SUMMARY.md"

[ -f "/home/ferradar/ros2_ws/README.md" ]
check_result "README.md (Project Root)"

[ -f "/home/ferradar/ros2_ws/DOCUMENTATION_INDEX.md" ]
check_result "DOCUMENTATION_INDEX.md"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/QUICK_START.md" ]
check_result "autonomous_patrol/QUICK_START.md"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/TECHNICAL_SPECS.md" ]
check_result "autonomous_patrol/TECHNICAL_SPECS.md"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/QUICK_START.md" ]
check_result "mono_depth_onnx/QUICK_START.md"

echo ""
echo "⚙️ CHECKING CONFIGURATION FILES"
echo "================================"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/config/autonomous_patrol_config.yaml" ]
check_result "autonomous_patrol_config.yaml"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/config/mono_depth_config.yaml" ]
check_result "mono_depth_config.yaml"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/config/mono_depth_config.yaml" ]
check_result "mono_depth_config.yaml"

echo ""
echo "🚀 CHECKING LAUNCH FILES"
echo "================================"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/launch/record_waypoints.launch.py" ]
check_result "autonomous_patrol/launch/record_waypoints.launch.py"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/launch/follow_waypoints.launch.py" ]
check_result "autonomous_patrol/launch/follow_waypoints.launch.py"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/launch/inference.launch.py" ]
check_result "mono_depth_onnx/launch/inference.launch.py"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/launch/full_pipeline.launch.py" ]
check_result "mono_depth_onnx/launch/full_pipeline.launch.py"

[ -f "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/launch/with_autonomy.launch.py" ]
check_result "mono_depth_onnx/launch/with_autonomy.launch.py"

echo ""
echo "📊 CHECKING DATA DIRECTORIES"
echo "================================"

[ -d "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/data" ]
check_result "autonomous_patrol/data/ directory exists"

[ -d "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/data" ]
check_result "mono_depth_onnx/data/ directory exists"

[ -d "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/models" ]
check_result "mono_depth_onnx/models/ directory exists"

[ -d "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/results" ]
check_result "autonomous_patrol/results/ directory exists"

[ -d "/home/ferradar/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx/results" ]
check_result "mono_depth_onnx/results/ directory exists"

echo ""
echo "🔧 CHECKING PYTHON DEPENDENCIES"
echo "================================"

# Check Python packages
python3 -c "import onnxruntime" 2>/dev/null
check_result "onnxruntime installed"

python3 -c "import cv2" 2>/dev/null
check_result "opencv (cv2) installed"

python3 -c "import numpy" 2>/dev/null
check_result "numpy installed"

python3 -c "import scipy" 2>/dev/null
check_result "scipy installed"

python3 -c "import yaml" 2>/dev/null
check_result "PyYAML installed"

python3 -c "import rclpy" 2>/dev/null
check_result "rclpy installed"

echo ""
echo "================================"
echo "📋 VERIFICATION SUMMARY"
echo "================================"

echo ""
echo -e "Total checks: $((PASS + FAIL))"
echo -e "${GREEN}Passed: $PASS${NC}"
echo -e "${RED}Failed: $FAIL${NC}"

if [ $FAIL -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✅ ALL CHECKS PASSED!${NC}"
    echo ""
    echo "Next steps:"
    echo "1. Download ONNX model:"
    echo "   cd ~/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx"
    echo "   python3 scripts/download_midas_model.py --model midas_v3_small"
    echo ""
    echo "2. Generate test images:"
    echo "   python3 scripts/generate_test_images.py"
    echo ""
    echo "3. Source and run:"
    echo "   cd ~/ros2_ws && source install/setup.bash"
    echo "   ros2 launch mono_depth_onnx full_pipeline.launch.py"
    echo ""
    exit 0
else
    echo ""
    echo -e "${RED}❌ SOME CHECKS FAILED!${NC}"
    echo "Please review the failed checks above and rebuild if needed:"
    echo "  cd ~/ros2_ws"
    echo "  colcon build --packages-select autonomous_patrol mono_depth_onnx"
    echo ""
    exit 1
fi

