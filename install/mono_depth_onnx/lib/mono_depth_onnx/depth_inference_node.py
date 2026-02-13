#!/usr/bin/env python3
"""
Depth Inference Node
Performs monocular depth estimation using ONNX Runtime with MiDaS model.

Processes RGB images and outputs estimated depth maps.
Depth values are in relative units (normalized 0-1).
"""

import os
import numpy as np
from typing import Optional, Tuple
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

try:
    import onnxruntime as ort
except ImportError:
    print("Please install onnxruntime: pip install onnxruntime")


class DepthInferenceNode(Node):
    """Performs depth estimation using ONNX Runtime."""

    def __init__(self):
        super().__init__('depth_inference')
        
        # Declare parameters
        self.declare_parameter('model_path', 'models/midas_v3_small.onnx')
        self.declare_parameter('model_name', 'midas_v3_small')
        self.declare_parameter('input_height', 256)
        self.declare_parameter('input_width', 256)
        self.declare_parameter('enable_gpu', False)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.model_name = self.get_parameter('model_name').value
        self.input_height = self.get_parameter('input_height').value
        self.input_width = self.get_parameter('input_width').value
        self.enable_gpu = self.get_parameter('enable_gpu').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Initialize ONNX Runtime session
        self.session = None
        self.input_name = None
        self.output_name = None
        
        if not self._initialize_model():
            self.get_logger().error('Failed to initialize depth model')
            return
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.depth_pub = self.create_publisher(Image, '/camera/depth_estimated', 10)
        self.depth_viz_pub = self.create_publisher(Image, '/camera/depth_colored', 10)
        
        # Performance metrics
        self.inference_times = []
        self.frame_count = 0
        
        self.get_logger().info(
            f'Depth Inference Node initialized\n'
            f'  Model: {self.model_name}\n'
            f'  Model path: {self.model_path}\n'
            f'  Input size: {self.input_width}x{self.input_height}\n'
            f'  GPU enabled: {self.enable_gpu}'
        )

    def _initialize_model(self) -> bool:
        """Initialize ONNX Runtime session."""
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Model not found: {self.model_path}')
            return False
        
        try:
            # Set provider based on GPU availability
            if self.enable_gpu:
                providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            else:
                providers = ['CPUExecutionProvider']
            
            self.get_logger().info(f'Loading ONNX model with providers: {providers}')
            
            self.session = ort.InferenceSession(
                self.model_path,
                providers=providers
            )
            
            # Get input and output names
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            
            input_shape = self.session.get_inputs()[0].shape
            output_shape = self.session.get_outputs()[0].shape
            
            self.get_logger().info(
                f'Model loaded successfully\n'
                f'  Input name: {self.input_name}, shape: {input_shape}\n'
                f'  Output name: {self.output_name}, shape: {output_shape}'
            )
            
            return True
        
        except Exception as e:
            self.get_logger().error(f'Failed to load ONNX model: {e}')
            return False

    def image_callback(self, msg: Image):
        """Process incoming RGB image."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run inference
            start_time = time.time()
            depth_map = self._estimate_depth(cv_image)
            inference_time = time.time() - start_time
            
            if depth_map is None:
                return
            
            # Track performance
            self.inference_times.append(inference_time)
            if len(self.inference_times) > 100:
                self.inference_times.pop(0)
            
            self.frame_count += 1
            
            # Publish depth map
            self._publish_depth(depth_map, msg.header)
            
            # Publish visualization
            self._publish_visualization(depth_map, msg.header)
            
            # Log performance
            if self.frame_count % 30 == 0:
                avg_time = np.mean(self.inference_times)
                max_time = np.max(self.inference_times)
                fps = 1.0 / avg_time if avg_time > 0 else 0
                
                self.get_logger().info(
                    f'Depth inference - Frame: {self.frame_count}, '
                    f'Time: {avg_time*1000:.1f}ms (max: {max_time*1000:.1f}ms), '
                    f'FPS: {fps:.1f}, '
                    f'Depth range: [{np.min(depth_map):.3f}, {np.max(depth_map):.3f}]'
                )
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def _estimate_depth(self, rgb_image: np.ndarray) -> Optional[np.ndarray]:
        """Estimate depth map from RGB image."""
        try:
            # Preprocess
            input_tensor = self._preprocess(rgb_image)
            
            # Run inference
            outputs = self.session.run([self.output_name], {self.input_name: input_tensor})
            depth_map = outputs[0]
            
            # Postprocess
            depth_map = self._postprocess(depth_map, rgb_image.shape)
            
            return depth_map
        
        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
            return None

    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image for model input."""
        # Resize
        resized = cv2.resize(image, (self.input_width, self.input_height),
                             interpolation=cv2.INTER_LINEAR)
        
        # Normalize (ImageNet normalization)
        normalized = resized.astype(np.float32) / 255.0
        
        # Mean and std normalization
        mean = np.array([0.485, 0.456, 0.406])
        std = np.array([0.229, 0.224, 0.225])
        normalized = (normalized - mean) / std
        
        # Change to CHW format for ONNX
        transposed = np.transpose(normalized, (2, 0, 1))
        
        # Add batch dimension
        batched = np.expand_dims(transposed, axis=0)
        
        return batched.astype(np.float32)

    def _postprocess(self, depth_map: np.ndarray, original_shape: Tuple) -> np.ndarray:
        """Postprocess depth map from model output."""
        # Remove batch dimension
        if len(depth_map.shape) == 4:
            depth_map = np.squeeze(depth_map, axis=0)
        
        # Remove channel dimension if needed
        if len(depth_map.shape) == 3:
            depth_map = np.squeeze(depth_map, axis=0)
        
        # Normalize to 0-1 range
        depth_min = np.min(depth_map)
        depth_max = np.max(depth_map)
        
        if depth_max - depth_min > 1e-6:
            depth_normalized = (depth_map - depth_min) / (depth_max - depth_min)
        else:
            depth_normalized = depth_map
        
        # Resize to original image size
        h, w = original_shape[:2]
        depth_resized = cv2.resize(depth_normalized, (w, h),
                                   interpolation=cv2.INTER_LINEAR)
        
        return depth_resized

    def _publish_depth(self, depth_map: np.ndarray, header):
        """Publish depth map as image (16-bit)."""
        # Convert to 16-bit format (0-65535)
        depth_uint16 = (depth_map * 65535).astype(np.uint16)
        
        # Create ROS Image message
        depth_msg = self.bridge.cv2_to_imgmsg(depth_uint16, encoding='mono16')
        depth_msg.header = header
        depth_msg.header.frame_id = 'camera'
        
        self.depth_pub.publish(depth_msg)

    def _publish_visualization(self, depth_map: np.ndarray, header):
        """Publish colored depth visualization."""
        # Normalize to 0-255
        depth_uint8 = (depth_map * 255).astype(np.uint8)
        
        # Apply colormap
        colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_TURBO)
        
        # Create ROS Image message
        viz_msg = self.bridge.cv2_to_imgmsg(colored, encoding='bgr8')
        viz_msg.header = header
        viz_msg.header.frame_id = 'camera'
        
        self.depth_viz_pub.publish(viz_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthInferenceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
