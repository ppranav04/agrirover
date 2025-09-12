import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class TaskCoordinator(Node):
    def __init__(self):
        super().__init__('task_node')

        self.status_pub = self.create_publisher(String, 'manipulator_status', 10)
        self.fruit_pub = self.create_publisher(String, 'fruit_status', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        #self.get_logger().info("Task coordinator node initialized.")

        # Wait for server and send goal
        self.nav_client.wait_for_server()
        self.send_nav_goal()

    def send_nav_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 2.0
        goal_msg.pose.pose.position.y = 1.0
        goal_msg.pose.pose.orientation.w = 1.0  # Facing forward

        self.get_logger().info("Sending navigation goal...")

        # Publish fruit detection status
        fruit_msg = String()
        fruit_msg.data = "Fruit detected"
        self.fruit_pub.publish(fruit_msg)
        self.get_logger().info(f"[fruit_status] {fruit_msg.data}")

        # Send goal
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # You can log intermediate feedback if you want:
        # self.get_logger().info(f"Current progress: {feedback}")

    def goal_result_callback(self, future):
        result = future.result().result
        status_msg = String()
        status_msg.data = "Ready for manipulation"
        self.status_pub.publish(status_msg)
        self.get_logger().info(f"[manipulator_status] {status_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = TaskCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

