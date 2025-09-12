#!/usr/bin/env python3

from threading import Thread
import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import arm

def deg_list(deg_values):
    return [math.radians(d) for d in deg_values]

class PickAndPlaceSequence(Node):
    def __init__(self):
        super().__init__('pick_and_place_sequence')

        callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=arm.joint_names(),
            base_link_name=arm.base_link_name(),
            end_effector_name=arm.end_effector_name(),
            group_name=arm.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        time.sleep(2)  # Wait for everything to initialize
        self.run_sequence()

    def run_sequence(self):
        poses = {
            "Home": [0, 0, 0, 0],
            "ready": [0, 90, 90, 90],
            "Pick": [0, -30,30, 15],
            "Place": [0, 0, 120, 30],
            "Back to Home": [0, 90, -120, -60]
        }

        for name, angles in poses.items():
            joint_positions_rad = deg_list(angles)
            self.get_logger().info(f"➡ Moving to {name} position: {angles}°")
            self.moveit2.move_to_configuration(joint_positions_rad)
            self.moveit2.wait_until_executed()
            self.get_logger().info(f"✅ Reached {name} position")

            # Hold 10 seconds after Pick and Place
            if name in ["Pick", "Place"]:
                self.get_logger().info(f"⏸ Holding at {name} position for 10 seconds...")
                time.sleep(10)

            time.sleep(1)

        self.get_logger().info("🏁 Pick-and-place sequence completed successfully!")
        rclpy.shutdown()

def main():
    rclpy.init()
    PickAndPlaceSequence()

if __name__ == '__main__':
    main()
