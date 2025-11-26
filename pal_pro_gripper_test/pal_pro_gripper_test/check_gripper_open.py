#!/usr/bin/env python3
# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class CheckGripperReverse(Node):
    def __init__(self):
        super().__init__('gripper_commander_reverse_node')

        self.pal_gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_left_controller/joint_trajectory',
            10)

        self.current_position = 0.07938
        self.step = -0.01
        self.min_position = 0.0

        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Node started. Closing gripper from 0.95 to 0.0.')

    def timer_callback(self):
        # Use a small tolerance (1e-6) for safe floating-point comparisons
        if self.current_position < self.min_position - 1e-6:
            self.get_logger().info('Minimum position reached. Stopping publishing.')
            self.timer.cancel()  # Stop the timer
            return

        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['gripper_left_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [self.current_position]
        point.time_from_start = Duration(sec=1, nanosec=0)

        traj_msg.points.append(point)

        self.pal_gripper_pub.publish(traj_msg)
        self.get_logger().info(f'Published gripper position: {self.current_position:.2f}')

        self.current_position += self.step


def main(args=None):
    rclpy.init(args=args)

    gripper_commander_node = CheckGripperReverse()

    try:
        rclpy.spin(gripper_commander_node)
    except KeyboardInterrupt:
        pass
    finally:
        gripper_commander_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
