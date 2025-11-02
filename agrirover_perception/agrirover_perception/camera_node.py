#!/usr/bin/env python3
"""
Enhanced Tomato Detection Camera Node
- Multi-camera support (Webcam, RealSense)
- Real-time YOLO-based tomato detection
- 3D pose estimation and publishing
- Smart detection timeout and home position triggering
- Confidence-based filtering
- Performance monitoring and statistics
- RViz visualization with markers
- Robust error handling
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray
import time
from collections import deque
from typing import List, Tuple, Optional

try:
    import pyrealsense2 as rs2
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False


class TomatoDetectionCameraNode(Node):
    def __init__(self):
        super().__init__('tomato_detection_camera_node')
        
        # ============================================================
        # DECLARE PARAMETERS
        # ============================================================
        
        self.declare_parameter('camera_type', 'webcam')  # 'webcam' or 'realsense'
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('z_distance', 0.2)  # Depth for webcam (meters)
        self.declare_parameter('model_path', '/home/pranav/agrirover_ws/src/yolov8n_tomato.pt')
        self.declare_parameter('detection_timeout', 3.0)  # Seconds before moving home
        self.declare_parameter('timer_period', 0.033)  # 30 Hz
        self.declare_parameter('confidence_threshold', 0.5)  # YOLO confidence threshold
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('max_detections_per_frame', 5)  # Limit detections per frame
        self.declare_parameter('enable_statistics', True)  # Track performance metrics
        self.declare_parameter('enable_visualization', True)  # Show detection window
        
        # ============================================================
        # GET PARAMETERS
        # ============================================================
        
        self.camera_type = self.get_parameter('camera_type').value
        self.camera_id = self.get_parameter('camera_id').value
        self.z_distance = self.get_parameter('z_distance').value
        self.model_path = self.get_parameter('model_path').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.timer_period = self.get_parameter('timer_period').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.max_detections = self.get_parameter('max_detections_per_frame').value
        self.enable_stats = self.get_parameter('enable_statistics').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        
        # ============================================================
        # INITIALIZE COMPONENTS
        # ============================================================
        
        self.bridge = CvBridge()
        
        # Load YOLO model with error handling
        try:
            self.yolo_model = YOLO(self.model_path)
            self.get_logger().info(f'✓ YOLO model loaded: {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'✗ Failed to load YOLO model: {e}')
            self.yolo_model = None
            return
        
        # Camera intrinsics
        self.camera_intrinsics = None
        self.fx = None
        self.fy = None
        self.ppx = None
        self.ppy = None
        
        # Detection tracking
        self.last_detection_time = time.time()
        self.detection_active = False
        self.home_position_triggered = False
        self.current_detections = []  # Store current frame detections
        
        # Performance statistics
        self.frame_count = 0
        self.detection_count = 0
        self.frame_times = deque(maxlen=30)  # Last 30 frame times
        self.last_stat_time = time.time()
        
        # ============================================================
        # PUBLISHERS
        # ============================================================
        
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'tomato/goal_pose', 10)
        self.detection_status_pub = self.create_publisher(Bool, 'tomato/detection_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'tomato/detection_markers', 10)
        
        # ============================================================
        # SUBSCRIBERS
        # ============================================================
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', 
            self.camera_info_callback, 10
        )
        
        # ============================================================
        # INITIALIZE CAMERA
        # ============================================================
        
        try:
            if self.camera_type.lower() == 'webcam':
                self.init_webcam()
            elif self.camera_type.lower() == 'realsense':
                if not REALSENSE_AVAILABLE:
                    self.get_logger().error('✗ RealSense library not available')
                    return
                self.init_realsense()
            else:
                self.get_logger().error(f'✗ Unknown camera type: {self.camera_type}')
                return
        except Exception as e:
            self.get_logger().error(f'✗ Failed to initialize camera: {e}')
            return
        
        # ============================================================
        # CREATE TIMER
        # ============================================================
        
        self.timer = self.create_timer(self.timer_period, self.process_frame)
        
        # ============================================================
        # LOG INITIALIZATION
        # ============================================================
        
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('  Enhanced Tomato Detection Camera Node')
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info(f'Camera type: {self.camera_type}')
        self.get_logger().info(f'Resolution: {self.frame_width}x{self.frame_height}')
        self.get_logger().info(f'FPS: {1/self.timer_period:.1f}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Detection timeout: {self.detection_timeout}s')
        if self.camera_type.lower() == 'webcam':
            self.get_logger().info(f'Webcam depth: {self.z_distance}m')
        self.get_logger().info(f'Statistics enabled: {self.enable_stats}')
        self.get_logger().info(f'Visualization enabled: {self.enable_viz}')
        self.get_logger().info('═══════════════════════════════════════')
    
    def init_webcam(self):
        """Initialize webcam with standard parameters."""
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'✗ Failed to open webcam (ID: {self.camera_id})')
            raise RuntimeError('Webcam initialization failed')
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Set camera intrinsics (typical webcam calibration)
        self.fx = 600.0
        self.fy = 600.0
        self.ppx = self.frame_width / 2.0
        self.ppy = self.frame_height / 2.0
        
        self.get_logger().info(f'✓ Webcam initialized (Device: {self.camera_id})')
    
    def init_realsense(self):
        """Initialize RealSense camera with depth sensor."""
        self.pipeline = rs2.pipeline()
        config = rs2.config()
        
        config.enable_stream(rs2.stream.color, self.frame_width, self.frame_height, rs2.format.bgr8, self.fps)
        config.enable_stream(rs2.stream.depth, self.frame_width, self.frame_height, rs2.format.z16, self.fps)
        
        profile = self.pipeline.start(config)
        
        color_stream = profile.get_stream(rs2.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        self.fx = intrinsics.fx
        self.fy = intrinsics.fy
        self.ppx = intrinsics.ppx
        self.ppy = intrinsics.ppy
        self.camera_intrinsics = intrinsics
        
        self.get_logger().info('✓ RealSense camera initialized')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Update camera intrinsics from camera_info message."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.ppx = msg.k[2]
        self.ppy = msg.k[5]
        
        self.get_logger().debug('Camera intrinsics updated from camera_info')
    
    def process_frame(self):
        """Main processing loop - capture, detect, and publish."""
        frame_start_time = time.time()
        
        try:
            # Capture frame
            frame, depth_frame = self.capture_frame()
            if frame is None:
                self.get_logger().warn('Failed to capture frame')
                return
            
            # Run YOLO detection
            if self.yolo_model is None:
                return
            
            detections = self.detect_objects(frame, depth_frame)
            self.current_detections = detections
            
            # Publish goal poses and markers
            self.publish_detections(detections)
            
            # Handle detection status and timeout
            self.handle_detection_status(len(detections) > 0, frame)
            
            # Draw detections on frame
            annotated_frame = self.annotate_frame(frame, detections)
            
            # Publish image
            self.publish_image(annotated_frame)
            
            # Display visualization
            if self.enable_viz:
                cv2.imshow('Tomato Detection', annotated_frame)
                cv2.waitKey(1)
            
            # Update statistics
            if self.enable_stats:
                frame_time = time.time() - frame_start_time
                self.frame_times.append(frame_time)
                self.frame_count += 1
                self.detection_count += len(detections)
                self.log_statistics()
        
        except Exception as e:
            self.get_logger().error(f'✗ Error in process_frame: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def capture_frame(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Capture frame from camera (webcam or RealSense)."""
        try:
            if self.camera_type.lower() == 'webcam':
                ret, frame = self.cap.read()
                if not ret:
                    return None, None
                return frame, None
            
            elif self.camera_type.lower() == 'realsense':
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    return None, None
                
                frame = np.asanyarray(color_frame.get_data())
                return frame, depth_frame
        
        except Exception as e:
            self.get_logger().error(f'✗ Frame capture error: {e}')
            return None, None
    
    def detect_objects(self, frame: np.ndarray, depth_frame=None) -> List[dict]:
        """Run YOLO detection and extract 3D positions."""
        detections = []
        
        try:
            # Run inference
            results = self.yolo_model(frame, conf=self.confidence_threshold, verbose=False)
            
            detection_count = 0
            for result in results:
                if detection_count >= self.max_detections:
                    break
                
                boxes = result.boxes
                for box in boxes:
                    if detection_count >= self.max_detections:
                        break
                    
                    xyxy = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf[0].cpu().numpy())
                    class_id = int(box.cls[0].cpu().numpy())
                    
                    # Calculate center
                    x1, y1, x2, y2 = xyxy
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    # Get depth
                    if self.camera_type.lower() == 'realsense' and depth_frame:
                        z = depth_frame.get_distance(center_x, center_y)
                    else:
                        z = self.z_distance
                    
                    # Convert to 3D
                    position_3d = self.pixel_to_3d(center_x, center_y, z)
                    
                    detection = {
                        'position': position_3d,
                        'confidence': confidence,
                        'class_id': class_id,
                        'bbox': (x1, y1, x2, y2),
                        'center': (center_x, center_y)
                    }
                    
                    detections.append(detection)
                    detection_count += 1
        
        except Exception as e:
            self.get_logger().error(f'✗ Detection error: {e}')
        
        return detections
    
    def pixel_to_3d(self, pixel_x: int, pixel_y: int, depth_z: float) -> List[float]:
        """Convert 2D pixel to 3D camera coordinates using pinhole model."""
        try:
            if self.camera_type.lower() == 'realsense' and self.camera_intrinsics:
                point_3d = rs2.rs2_deproject_pixel_to_point(
                    self.camera_intrinsics, [pixel_x, pixel_y], depth_z
                )
                return list(point_3d)
            else:
                # Pinhole camera model
                x = (pixel_x - self.ppx) * depth_z / self.fx
                y = (pixel_y - self.ppy) * depth_z / self.fy
                z = depth_z
                return [x, y, z]
        except Exception as e:
            self.get_logger().error(f'✗ 3D conversion error: {e}')
            return [0.0, 0.0, 0.0]
    
    def publish_detections(self, detections: List[dict]):
        """Publish goal poses and markers for all detections."""
        # Publish goal poses
        for detection in detections:
            self.publish_goal_pose(detection)
        
        # Publish markers
        self.publish_markers(detections)
    
    def publish_goal_pose(self, detection: dict):
        """Publish single goal pose."""
        position = detection['position']
        confidence = detection['confidence']
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_head_link'
        
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        pose_msg.pose.orientation.w = 1.0
        
        self.goal_pose_pub.publish(pose_msg)
        
        self.get_logger().debug(
            f'Goal pose: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] '
            f'(conf={confidence:.2f})'
        )
    
    def publish_markers(self, detections: List[dict]):
        """Publish RViz markers for visualization."""
        marker_array = MarkerArray()
        
        # Clear old markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Create markers for each detection
        for i, detection in enumerate(detections):
            position = detection['position']
            confidence = detection['confidence']
            
            # Sphere marker
            marker = Marker()
            marker.header.frame_id = 'camera_head_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(position[0])
            marker.pose.position.y = float(position[1])
            marker.pose.position.z = float(position[2])
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            # Color based on confidence
            if confidence > 0.8:
                marker.color.r = 0.0
                marker.color.g = 1.0  # Green
                marker.color.b = 0.0
            elif confidence > 0.6:
                marker.color.r = 1.0
                marker.color.g = 1.0  # Yellow
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0  # Red
                marker.color.b = 0.0
            
            marker.color.a = 0.8
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            
            marker_array.markers.append(marker)
            
            # Text marker
            text_marker = Marker()
            text_marker.header.frame_id = 'camera_head_link'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.id = 1000 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(position[0])
            text_marker.pose.position.y = float(position[1])
            text_marker.pose.position.z = float(position[2]) + 0.1
            text_marker.pose.orientation.w = 1.0
            
            text_marker.text = f'Conf: {confidence:.2f}'
            text_marker.scale.z = 0.03
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
    
    def handle_detection_status(self, detections_found: bool, frame: np.ndarray):
        """Handle detection timeout and home position triggering."""
        current_time = time.time()
        
        if detections_found:
            self.last_detection_time = current_time
            self.detection_active = True
            self.home_position_triggered = False
            
            status_msg = Bool()
            status_msg.data = True
            self.detection_status_pub.publish(status_msg)
            
            cv2.putText(frame, "STATUS: DETECTION ACTIVE", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        else:
            time_since_detection = current_time - self.last_detection_time
            
            if time_since_detection > self.detection_timeout and not self.home_position_triggered:
                self.get_logger().warn(
                    f'No tomato detected for {self.detection_timeout}s. Moving to home.'
                )
                
                status_msg = Bool()
                status_msg.data = False
                self.detection_status_pub.publish(status_msg)
                
                self.home_position_triggered = True
                self.detection_active = False
                
                cv2.putText(frame, "STATUS: NO DETECTION - MOVING HOME", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            elif time_since_detection <= self.detection_timeout:
                cv2.putText(frame, f"STATUS: SEARCHING ({time_since_detection:.1f}s)", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
    
    def annotate_frame(self, frame: np.ndarray, detections: List[dict]) -> np.ndarray:
        """Draw detections and info on frame."""
        annotated = frame.copy()
        
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            confidence = detection['confidence']
            position = detection['position']
            center_x, center_y = detection['center']
            
            # Draw bounding box
            cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(annotated, center_x, center_y, 5, (0, 0, 255), -1)
            
            # Draw label
            label = f'Tomato {confidence:.2f}'
            cv2.putText(annotated, label, (int(x1), int(y1)-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw 3D position
            pos_label = f'3D: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})'
            cv2.putText(annotated, pos_label, (int(x1), int(y2)+20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 0), 1)
        
        # Draw statistics
        if self.enable_stats:
            fps = len(self.frame_times) / (sum(self.frame_times) + 1e-6)
            stats_text = f'FPS: {fps:.1f} | Detections: {len(detections)} | Total: {self.detection_count}'
            cv2.putText(annotated, stats_text, (10, annotated.shape[0]-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return annotated
    
    def publish_image(self, frame: np.ndarray):
        """Publish annotated frame."""
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_head_link'
        self.image_pub.publish(img_msg)
    
    def log_statistics(self):
        """Log performance statistics periodically."""
        current_time = time.time()
        if current_time - self.last_stat_time > 5.0:  # Log every 5 seconds
            avg_frame_time = np.mean(self.frame_times) * 1000 if self.frame_times else 0
            fps = 1000.0 / avg_frame_time if avg_frame_time > 0 else 0
            
            self.get_logger().info(
                f'Stats: FPS={fps:.1f} | Frames={self.frame_count} | '
                f'Detections={self.detection_count} | Avg time={avg_frame_time:.1f}ms'
            )
            self.last_stat_time = current_time
    
    def cleanup(self):
        """Clean up resources on shutdown."""
        try:
            if self.camera_type.lower() == 'webcam':
                if hasattr(self, 'cap'):
                    self.cap.release()
            elif self.camera_type.lower() == 'realsense':
                if hasattr(self, 'pipeline'):
                    self.pipeline.stop()
            
            cv2.destroyAllWindows()
            self.get_logger().info('✓ Camera resources cleaned up')
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TomatoDetectionCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
