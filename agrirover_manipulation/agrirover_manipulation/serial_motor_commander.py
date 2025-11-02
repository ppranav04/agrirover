#!/usr/bin/env python3
"""
Enhanced Serial Motor Commander
- Graceful degradation when hardware unavailable
- Connection retry logic
- Movement queue management
- Better error recovery
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import serial
import math
import time
import numpy as np
from threading import Lock, Thread
from queue import Queue, Empty


class EnhancedSerialMotorCommander(Node):
    def __init__(self):
        super().__init__('serial_motor_commander')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('interpolation_steps', 20)
        self.declare_parameter('movement_duration', 2.0)
        self.declare_parameter('use_easing', True)
        self.declare_parameter('enable_hardware', False)
        self.declare_parameter('connection_retry_interval', 5.0)
        self.declare_parameter('max_connection_attempts', -1)  # -1 = infinite
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.interpolation_steps = self.get_parameter('interpolation_steps').value
        self.movement_duration = self.get_parameter('movement_duration').value
        self.use_easing = self.get_parameter('use_easing').value
        self.enable_hardware = self.get_parameter('enable_hardware').value
        self.retry_interval = self.get_parameter('connection_retry_interval').value
        self.max_attempts = self.get_parameter('max_connection_attempts').value
        
        # Serial connection
        self.ser = None
        self.connection_lock = Lock()
        self.is_connected = False
        self.connection_attempts = 0
        
        # Movement tracking
        self.current_positions = None
        self.movement_lock = Lock()
        self.movement_queue = Queue(maxsize=5)
        self.is_moving = False
        
        # Publishers
        self.connection_status_pub = self.create_publisher(
            Bool, 'serial/connection_status', 10
        )
        
        # Subscribers
        self.servo_sub = self.create_subscription(
            Float32MultiArray,
            'servo_angles',
            self.servo_callback,
            10
        )
        
        # Initialize connection
        if self.enable_hardware:
            self.connect_to_serial()
            
            # Retry timer if connection fails
            if not self.is_connected:
                self.retry_timer = self.create_timer(
                    self.retry_interval,
                    self.retry_connection
                )
        else:
            self.get_logger().info('Hardware disabled - running in simulation mode')
        
        # Start movement worker thread
        self.movement_thread = Thread(target=self.movement_worker, daemon=True)
        self.movement_thread.start()
        
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('  Enhanced Serial Motor Commander')
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info(f'Serial port: {self.serial_port}')
        self.get_logger().info(f'Baud rate: {self.baud_rate}')
        self.get_logger().info(f'Hardware enabled: {self.enable_hardware}')
        self.get_logger().info(f'Interpolation: {self.interpolation_steps} steps')
        self.get_logger().info(f'Duration: {self.movement_duration}s')
    
    def connect_to_serial(self):
        """Attempt to connect to serial port"""
        try:
            with self.connection_lock:
                if self.ser is not None:
                    self.ser.close()
                
                self.ser = serial.Serial(
                    self.serial_port,
                    self.baud_rate,
                    timeout=1
                )
                self.is_connected = True
                self.connection_attempts = 0
                
                self.get_logger().info(
                    f'✓ Connected to {self.serial_port} at {self.baud_rate} baud'
                )
                
                # Publish connection status
                status_msg = Bool()
                status_msg.data = True
                self.connection_status_pub.publish(status_msg)
                
                return True
                
        except serial.SerialException as e:
            self.is_connected = False
            self.get_logger().error(f'✗ Failed to connect: {e}')
            
            # Publish connection status
            status_msg = Bool()
            status_msg.data = False
            self.connection_status_pub.publish(status_msg)
            
            return False
    
    def retry_connection(self):
        """Retry serial connection periodically"""
        if self.is_connected:
            self.retry_timer.cancel()
            return
        
        self.connection_attempts += 1
        
        if self.max_attempts > 0 and self.connection_attempts > self.max_attempts:
            self.get_logger().error(
                f'✗ Max connection attempts ({self.max_attempts}) reached'
            )
            self.retry_timer.cancel()
            return
        
        self.get_logger().info(
            f'Retrying connection (attempt {self.connection_attempts})...'
        )
        
        if self.connect_to_serial():
            self.retry_timer.cancel()
    
    def ease_in_out_cubic(self, t):
        """Cubic ease-in-out easing function"""
        if t < 0.5:
            return 4 * t * t * t
        else:
            return 1 - pow(-2 * t + 2, 3) / 2
    
    def interpolate_trajectory(self, start_positions, target_positions):
        """Generate smooth interpolated trajectory"""
        trajectory = []
        
        for i in range(self.interpolation_steps + 1):
            t = i / self.interpolation_steps
            
            if self.use_easing:
                t_eased = self.ease_in_out_cubic(t)
            else:
                t_eased = t
            
            interpolated_position = []
            for start, target in zip(start_positions, target_positions):
                pos = start + (target - start) * t_eased
                interpolated_position.append(pos)
            
            trajectory.append(interpolated_position)
        
        return trajectory
    
    def servo_callback(self, msg: Float32MultiArray):
        """Handle incoming servo angle commands"""
        target_positions = list(msg.data)
        
        # Initialize current positions on first command
        if self.current_positions is None:
            self.current_positions = target_positions
            self.get_logger().info('Initialized current positions')
        
        # Add to movement queue
        try:
            self.movement_queue.put_nowait(target_positions)
            self.get_logger().debug(f'Added movement to queue (size: {self.movement_queue.qsize()})')
        except:
            self.get_logger().warn('Movement queue full, dropping command')
    
    def movement_worker(self):
        """Worker thread for executing movements"""
        while rclpy.ok():
            try:
                # Get next movement from queue (blocking)
                target_positions = self.movement_queue.get(timeout=0.1)
                
                with self.movement_lock:
                    self.is_moving = True
                    self.execute_movement(target_positions)
                    self.is_moving = False
                
                self.movement_queue.task_done()
                
            except Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Movement worker error: {e}')
    
    def execute_movement(self, target_positions):
        """Execute smooth movement to target positions"""
        if not self.enable_hardware:
            # Simulation mode
            self.get_logger().info(
                f'[SIM] Moving to: {[f"{p:.2f}" for p in target_positions]}'
            )
            time.sleep(self.movement_duration)
            self.current_positions = target_positions
            return
        
        if not self.is_connected or self.ser is None:
            self.get_logger().error('Serial port not connected')
            return
        
        try:
            # Generate trajectory
            trajectory = self.interpolate_trajectory(
                self.current_positions, target_positions
            )
            
            step_delay = self.movement_duration / self.interpolation_steps
            
            self.get_logger().info(
                f'Executing movement: {len(trajectory)} steps over {self.movement_duration}s'
            )
            
            # Send each position
            for i, position_rad in enumerate(trajectory):
                # Convert to degrees
                angles_deg = [
                    max(0, min(180, int(math.degrees(angle))))
                    for angle in position_rad
                ]
                
                # Send command
                command_str = ','.join(map(str, angles_deg)) + '\n'
                
                with self.connection_lock:
                    if self.ser is not None:
                        self.ser.write(command_str.encode('utf-8'))
                
                # Log progress
                if i % 5 == 0 or i == len(trajectory) - 1:
                    self.get_logger().debug(
                        f'Step {i}/{len(trajectory)}: {angles_deg}'
                    )
                
                # Wait
                if i < len(trajectory) - 1:
                    time.sleep(step_delay)
            
            # Update current position
            self.current_positions = target_positions
            self.get_logger().info('✓ Movement complete')
            
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error during movement: {e}')
            self.is_connected = False
            
            # Try to reconnect
            if self.enable_hardware:
                self.connect_to_serial()
        
        except Exception as e:
            self.get_logger().error(f'Movement execution error: {e}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.ser is not None:
            try:
                self.ser.close()
                self.get_logger().info('Serial port closed')
            except:
                pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EnhancedSerialMotorCommander()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
