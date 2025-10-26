import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import math

class SerialMotorCommander(Node):
    def __init__(self):
        super().__init__('serial_motor_commander')
        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyUSB0').get_parameter_value().string_value
        self.baud_rate = self.declare_parameter('baud_rate', 9600).get_parameter_value().integer_value
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Opened serial port {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None
        
        self.subscriber = self.create_subscription(
            Float32MultiArray,
            'servo_angles',
            self.listener_callback,
            10)

    def listener_callback(self, msg: Float32MultiArray):
        if self.ser is None:
            self.get_logger().error("Serial port not open")
            return
        
        # Convert radians to 0-180 servo angle degrees
        try:
            angles_deg = [max(0, min(180, int(math.degrees(angle)))) for angle in msg.data]
            command_str = ','.join(map(str, angles_deg)) + '\n'
            self.ser.write(command_str.encode('utf-8'))
            self.get_logger().info(f"Sent to Arduino: {command_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error in sending command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialMotorCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
