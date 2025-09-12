import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import math


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender_node')

        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.fruit_status_pub = self.create_publisher(String, '/fruit_status', 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        self.timer_initial = self.create_timer(5.0, self.send_initial_pose)
        self.timer_goal = None

        self.initial_sent = False
        self.goal_sent = False
        self.pick_started = False

        # Goal coordinates
        self.goal_x = -1.5
        self.goal_y = -1.01
        self.goal_tolerance = 0.25  # meters

    def send_initial_pose(self):
        if self.initial_sent:
            return

        start = PoseWithCovarianceStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.pose.position.x = 1.5
        start.pose.pose.position.y = -3.25
        start.pose.pose.orientation.w = 1.0

        self.initial_pub.publish(start)
        self.get_logger().info("✅ Initial pose published.")
        self.initial_sent = True

        self.timer_goal = self.create_timer(5.0, self.send_goal_pose)

    def send_goal_pose(self):
        if self.goal_sent:
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info("✅ Goal pose published.")
        self.goal_sent = True

        # Publish fruit detected status
        self.fruit_status_pub.publish(String(data="fruit detected"))
        self.get_logger().info("📢 Published: fruit detected")

    def pose_callback(self, msg):
        if self.pick_started:
            return

        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        dist = math.hypot(curr_x - self.goal_x, curr_y - self.goal_y)
        if dist <= self.goal_tolerance:
            self.fruit_status_pub.publish(String(data="starting fruit picking"))
            self.get_logger().info("📢 Published: starting fruit picking")
            self.pick_started = True
            self.get_logger().info("✅ Done. Shutting down...")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()

