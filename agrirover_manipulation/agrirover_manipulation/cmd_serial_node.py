import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_serial_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.get_logger().info("Serial connection established.")

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        command = f"{linear:.2f},{angular:.2f}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent to Arduino: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
