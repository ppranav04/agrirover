#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs2  # For RealSense camera (future)
from ultralytics import YOLO  # YOLOv8 for tomato detection

class TomatoDetectionCameraNode(Node):
    def __init__(self):
        super().__init__('tomato_detection_camera_node')
        
        # Parameters
        self.declare_parameter('camera_type', 'webcam')  # 'webcam' or 'realsense'
        self.declare_parameter('camera_id', 0)  # For webcam
        self.declare_parameter('z_distance', 0.5)  # User-provided Z value (meters)
        self.declare_parameter('model_path', '/home/pranav/agrirover_ws/src/yolov8n_tomato.pt')  # Path to trained YOLOv8 model
        
        self.camera_type = self.get_parameter('camera_type').value
        self.camera_id = self.get_parameter('camera_id').value
        self.z_distance = self.get_parameter('z_distance').value
        
        # Initialize components
        self.bridge = CvBridge()
        self.yolo_model = YOLO(self.get_parameter('model_path').value)
        
        # Camera intrinsics (will be updated when CameraInfo is received)
        self.camera_intrinsics = None
        self.fx = None  # Focal length x
        self.fy = None  # Focal length y
        self.ppx = None  # Principal point x
        self.ppy = None  # Principal point y
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'tomato/goal_pose', 10)
        
        # Subscribers (for RealSense depth info)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', 
            self.camera_info_callback, 10
        )
        
        # Initialize camera based on type
        if self.camera_type == 'webcam':
            self.init_webcam()
        elif self.camera_type == 'realsense':
            self.init_realsense()
        
        # Timer for processing frames
        self.timer = self.create_timer(0.2, self.process_frame)  # 10 Hz
        
        self.get_logger().info(f'Camera node initialized with {self.camera_type}')
    
    def init_webcam(self):
        '''Initialize laptop webcam using OpenCV'''
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open webcam')
            return
        
        # Set webcam properties (optional)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Approximate camera intrinsics for webcam (should be calibrated)
        self.fx = 600.0  # Approximate focal length (pixels)
        self.fy = 600.0
        self.ppx = 320.0  # Image center
        self.ppy = 240.0
        
        self.get_logger().info('Webcam initialized')
    
    def init_realsense(self):
        '''Initialize Intel RealSense D435 camera'''
        self.pipeline = rs2.pipeline()
        config = rs2.config()
        
        # Enable streams
        config.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)
        config.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
        
        # Start pipeline
        profile = self.pipeline.start(config)
        
        # Get camera intrinsics
        color_stream = profile.get_stream(rs2.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        self.fx = intrinsics.fx
        self.fy = intrinsics.fy
        self.ppx = intrinsics.ppx
        self.ppy = intrinsics.ppy
        self.camera_intrinsics = intrinsics
        
        self.get_logger().info('RealSense camera initialized')
    
    def camera_info_callback(self, msg):
        '''Update camera intrinsics from CameraInfo message'''
        self.fx = msg.k
        self.fy = msg.k
        self.ppx = msg.k
        self.ppy = msg.k
    
    def process_frame(self):
        # Capture frame based on camera type
        if self.camera_type == 'webcam':
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Failed to capture frame from webcam')
                return
            depth_frame = None

        elif self.camera_type == 'realsense':
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame:
                self.get_logger().warn('Failed to capture frame from RealSense')
                return

            frame = np.asanyarray(color_frame.get_data())

    # Detect tomatoes using YOLOv8
        results = self.yolo_model(frame, conf=0.5)
        
        # Process detections
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Extract bounding box coordinates correctly
                xyxy = box.xyxy[0].cpu().numpy()  # shape (4,) - single box coords
                confidence = box.conf[0].cpu().numpy()  # scalar confidence
                class_id = int(box.cls[0].cpu().numpy())  # scalar class index

                x1, y1, x2, y2 = xyxy

                # Calculate center
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                # Get depth
                if self.camera_type == 'realsense' and depth_frame:
                    z = depth_frame.get_distance(center_x, center_y)
                else:
                    z = self.z_distance

                # Convert 2D pixel to 3D camera coords
                goal_pose = self.pixel_to_3d(center_x, center_y, z)

                # Publish goal pose
                self.publish_goal_pose(goal_pose, confidence, class_id)

                # Draw detections
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                label = f'Tomato {confidence:.2f} Pose: ({goal_pose[0]:.3f}, {goal_pose[1]:.3f}, {goal_pose[2]:.3f})'
                cv2.putText(frame, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish image
        self.publish_image(frame)
        
        # Display frame (optional, for debugging)
        cv2.imshow('Tomato Detection', frame)
        cv2.waitKey(1)
    
    def pixel_to_3d(self, pixel_x, pixel_y, depth_z):
        '''
        Convert 2D pixel coordinates to 3D camera coordinates
        
        Formula (Pinhole Camera Model):
            X = (u - ppx) * Z / fx
            Y = (v - ppy) * Z / fy
            Z = depth
        
        Args:
            pixel_x: X coordinate in image (pixels)
            pixel_y: Y coordinate in image (pixels)
            depth_z: Depth/distance in meters
        
        Returns:
            [x, y, z]: 3D coordinates in camera frame (meters)
        '''
        
        if self.camera_type == 'realsense' and self.camera_intrinsics:
            # Use RealSense built-in deprojection
            point_3d = rs2.rs2_deproject_pixel_to_point(
                self.camera_intrinsics, [pixel_x, pixel_y], depth_z
            )
            return point_3d
        
        else:
            # Manual deprojection using pinhole camera model
            x = (pixel_x - self.ppx) * depth_z / self.fx
            y = (pixel_y - self.ppy) * depth_z / self.fy
            z = depth_z
            
            return [x, y, z]
    
    def publish_goal_pose(self, position, confidence, class_id):
        '''Publish goal pose as PoseStamped message'''
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_link'
        
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])

        
        # Orientation (can be adjusted based on requirements)
        pose_msg.pose.orientation.w = 1.0
        
        self.goal_pose_pub.publish(pose_msg)
        
        self.get_logger().info(
            f'Goal Pose Published: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}, conf={confidence:.2f}'

        )
    
    def publish_image(self, frame):
        '''Publish image as ROS2 Image message'''
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_link'
        self.image_pub.publish(img_msg)
    
    def cleanup(self):
        '''Cleanup resources'''
        if self.camera_type == 'webcam':
            self.cap.release()
        elif self.camera_type == 'realsense':
            self.pipeline.stop()
        cv2.destroyAllWindows()

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
