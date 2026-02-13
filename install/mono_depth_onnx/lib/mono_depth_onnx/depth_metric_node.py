#!/usr/bin/env python3
"""
Depth Metric Node
Calculates simple environmental metrics from depth map.
Examples: minimum frontal depth, average depth, obstacle detection.

Implements filtering to reduce outliers and noise.
"""

import numpy as np
from typing import Optional, Tuple
import json
import os
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
from scipy import ndimage


class DepthMetricNode(Node):
    """Calculate metrics from depth maps."""

    def __init__(self):
        super().__init__('depth_metric')
        
        # Declare parameters
        self.declare_parameter('roi_height_ratio', 0.3)  # Top 30% of image
        self.declare_parameter('roi_width_ratio', 0.5)    # Center 50% of image width
        self.declare_parameter('min_depth_threshold', 0.1)
        self.declare_parameter('max_depth_threshold', 0.9)
        self.declare_parameter('outlier_filter_type', 'median')  # 'median', 'percentile', 'iqr'
        self.declare_parameter('outlier_filter_size', 5)
        self.declare_parameter('safety_depth_threshold', 0.3)  # Below this = obstacle
        self.declare_parameter('obstacle_area_ratio', 0.1)  # Ratio of ROI that must be obstacle
        self.declare_parameter('metrics_history_size', 100)
        self.declare_parameter('results_directory', 'results')
        
        # Get parameters
        self.roi_h_ratio = self.get_parameter('roi_height_ratio').value
        self.roi_w_ratio = self.get_parameter('roi_width_ratio').value
        self.min_depth_th = self.get_parameter('min_depth_threshold').value
        self.max_depth_th = self.get_parameter('max_depth_threshold').value
        self.filter_type = self.get_parameter('outlier_filter_type').value
        self.filter_size = self.get_parameter('outlier_filter_size').value
        self.safety_th = self.get_parameter('safety_depth_threshold').value
        self.obstacle_ratio = self.get_parameter('obstacle_area_ratio').value
        self.history_size = self.get_parameter('metrics_history_size').value
        self.results_dir = self.get_parameter('results_directory').value
        
        # Create results directory
        os.makedirs(self.results_dir, exist_ok=True)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Metrics history
        self.min_depth_history = deque(maxlen=self.history_size)
        self.mean_depth_history = deque(maxlen=self.history_size)
        self.obstacle_detected_history = deque(maxlen=self.history_size)
        self.frame_count = 0
        self.start_time = datetime.now()
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth_estimated',
            self.depth_callback,
            10
        )
        
        # Publishers
        self.min_depth_pub = self.create_publisher(Float32, '/depth_metric/min_frontal_depth', 10)
        self.mean_depth_pub = self.create_publisher(Float32, '/depth_metric/mean_depth', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/depth_metric/obstacle_detected', 10)
        self.roi_viz_pub = self.create_publisher(Image, '/depth_metric/roi_visualization', 10)
        
        self.get_logger().info(
            f'Depth Metric Node initialized\n'
            f'  ROI: top {self.roi_h_ratio*100:.0f}%, center {self.roi_w_ratio*100:.0f}%\n'
            f'  Filter: {self.filter_type}\n'
            f'  Safety threshold: {self.safety_th}\n'
            f'  Obstacle ratio: {self.obstacle_ratio*100:.1f}%'
        )

    def depth_callback(self, msg: Image):
        """Process depth map and calculate metrics."""
        try:
            # Convert ROS Image to OpenCV (16-bit)
            depth_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            
            # Convert to float (0-1 range)
            depth_float = depth_cv.astype(np.float32) / 65535.0
            
            # Extract ROI
            roi = self._extract_roi(depth_float)
            
            # Filter outliers
            roi_filtered = self._apply_outlier_filter(roi)
            
            # Calculate metrics
            min_depth = self._calculate_min_depth(roi_filtered)
            mean_depth = self._calculate_mean_depth(roi_filtered)
            obstacle_detected = self._detect_obstacle(roi_filtered)
            
            # Store in history
            self.min_depth_history.append(min_depth)
            self.mean_depth_history.append(mean_depth)
            self.obstacle_detected_history.append(obstacle_detected)
            
            self.frame_count += 1
            
            # Publish metrics
            self._publish_min_depth(min_depth)
            self._publish_mean_depth(mean_depth)
            self._publish_obstacle_status(obstacle_detected)
            
            # Publish ROI visualization
            self._publish_roi_visualization(depth_float, roi_filtered)
            
            # Log periodically
            if self.frame_count % 30 == 0:
                self._log_metrics(min_depth, mean_depth, obstacle_detected)
        
        except Exception as e:
            self.get_logger().error(f'Error processing depth: {e}')

    def _extract_roi(self, depth_map: np.ndarray) -> np.ndarray:
        """Extract Region of Interest (ROI) from depth map."""
        h, w = depth_map.shape
        
        # Frontal region: top roi_height_ratio%, center roi_width_ratio%
        roi_height = int(h * self.roi_h_ratio)
        roi_width = int(w * self.roi_w_ratio)
        roi_x_start = int((w - roi_width) / 2)
        roi_y_start = 0
        
        roi = depth_map[
            roi_y_start:roi_y_start + roi_height,
            roi_x_start:roi_x_start + roi_width
        ]
        
        return roi

    def _apply_outlier_filter(self, roi: np.ndarray) -> np.ndarray:
        """Apply outlier filtering to ROI."""
        if self.filter_type == 'median':
            return ndimage.median_filter(roi, size=self.filter_size)
        
        elif self.filter_type == 'percentile':
            # Remove extreme values using percentile clipping
            p5 = np.percentile(roi, 5)
            p95 = np.percentile(roi, 95)
            clipped = np.clip(roi, p5, p95)
            return ndimage.median_filter(clipped, size=self.filter_size)
        
        elif self.filter_type == 'iqr':
            # Interquartile range filtering
            q1 = np.percentile(roi, 25)
            q3 = np.percentile(roi, 75)
            iqr = q3 - q1
            lower = q1 - 1.5 * iqr
            upper = q3 + 1.5 * iqr
            clipped = np.clip(roi, lower, upper)
            return ndimage.median_filter(clipped, size=self.filter_size)
        
        else:
            return roi

    def _calculate_min_depth(self, roi: np.ndarray) -> float:
        """Calculate minimum depth (closest obstacle)."""
        valid_values = roi[
            (roi >= self.min_depth_th) & (roi <= self.max_depth_th)
        ]
        
        if len(valid_values) == 0:
            return 0.0
        
        return float(np.min(valid_values))

    def _calculate_mean_depth(self, roi: np.ndarray) -> float:
        """Calculate mean depth."""
        valid_values = roi[
            (roi >= self.min_depth_th) & (roi <= self.max_depth_th)
        ]
        
        if len(valid_values) == 0:
            return 0.0
        
        return float(np.mean(valid_values))

    def _detect_obstacle(self, roi: np.ndarray) -> bool:
        """Detect if obstacle is too close."""
        # Count pixels below safety threshold
        obstacles = roi < self.safety_th
        obstacle_ratio = np.sum(obstacles) / roi.size
        
        return bool(obstacle_ratio > self.obstacle_ratio)

    def _publish_min_depth(self, value: float):
        """Publish minimum depth metric."""
        msg = Float32(data=value)
        self.min_depth_pub.publish(msg)

    def _publish_mean_depth(self, value: float):
        """Publish mean depth metric."""
        msg = Float32(data=value)
        self.mean_depth_pub.publish(msg)

    def _publish_obstacle_status(self, detected: bool):
        """Publish obstacle detection status."""
        msg = Bool(data=detected)
        self.obstacle_pub.publish(msg)

    def _publish_roi_visualization(self, depth_map: np.ndarray, roi_filtered: np.ndarray):
        """Publish ROI visualization."""
        try:
            # Create visualization
            h, w = depth_map.shape
            roi_height = int(h * self.roi_h_ratio)
            roi_width = int(w * self.roi_w_ratio)
            roi_x_start = int((w - roi_width) / 2)
            
            # Normalize depth for visualization
            depth_uint8 = (depth_map * 255).astype(np.uint8)
            colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_TURBO)
            
            # Draw ROI rectangle
            cv2.rectangle(colored,
                         (roi_x_start, 0),
                         (roi_x_start + roi_width, roi_height),
                         (0, 255, 0), 2)
            
            # Add text with metrics
            min_depth = self.min_depth_history[-1] if self.min_depth_history else 0
            mean_depth = self.mean_depth_history[-1] if self.mean_depth_history else 0
            
            cv2.putText(colored, f'Min: {min_depth:.3f}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(colored, f'Mean: {mean_depth:.3f}', (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Convert to ROS Image
            viz_msg = self.bridge.cv2_to_imgmsg(colored, encoding='bgr8')
            viz_msg.header.frame_id = 'camera'
            viz_msg.header.stamp = self.get_clock().now().to_msg()
            
            self.roi_viz_pub.publish(viz_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error publishing ROI visualization: {e}')

    def _log_metrics(self, min_depth: float, mean_depth: float, obstacle: bool):
        """Log metrics to console and file."""
        elapsed = (datetime.now() - self.start_time).total_seconds()
        
        status = "🚨 OBSTACLE" if obstacle else "✓ Clear"
        
        self.get_logger().info(
            f'Depth Metrics (Frame {self.frame_count}, {elapsed:.1f}s)\n'
            f'  Min depth: {min_depth:.4f}\n'
            f'  Mean depth: {mean_depth:.4f}\n'
            f'  Status: {status}'
        )
        
        # Save to JSON periodically
        if self.frame_count % 100 == 0:
            self._save_metrics_to_file()

    def _save_metrics_to_file(self):
        """Save collected metrics to JSON file."""
        try:
            metrics = {
                'timestamp': datetime.now().isoformat(),
                'total_frames': self.frame_count,
                'min_depth_stats': {
                    'current': self.min_depth_history[-1] if self.min_depth_history else None,
                    'mean': np.mean(list(self.min_depth_history)) if self.min_depth_history else None,
                    'max': np.max(list(self.min_depth_history)) if self.min_depth_history else None,
                    'min': np.min(list(self.min_depth_history)) if self.min_depth_history else None,
                },
                'mean_depth_stats': {
                    'current': self.mean_depth_history[-1] if self.mean_depth_history else None,
                    'mean': np.mean(list(self.mean_depth_history)) if self.mean_depth_history else None,
                    'max': np.max(list(self.mean_depth_history)) if self.mean_depth_history else None,
                    'min': np.min(list(self.mean_depth_history)) if self.mean_depth_history else None,
                },
                'obstacle_detections': int(np.sum(list(self.obstacle_detected_history))),
                'obstacle_ratio': float(np.sum(list(self.obstacle_detected_history)) / len(self.obstacle_detected_history)) if self.obstacle_detected_history else 0,
            }
            
            metrics_file = os.path.join(self.results_dir, 'depth_metrics.json')
            with open(metrics_file, 'w') as f:
                json.dump(metrics, f, indent=2)
        
        except Exception as e:
            self.get_logger().error(f'Failed to save metrics: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthMetricNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
