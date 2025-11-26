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


class CheckGripper(Node):
    def __init__(self):
        super().__init__('gripper_commander_node')

        self.pal_gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_left_controller/joint_trajectory',
            10)

        self.current_position = 0.0
        self.step = 0.01
        self.max_position = 0.07938

        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Node started. Publishing a new gripper position every 5 seconds.')

    def timer_callback(self):
        # Check whether the maximum position has been exceeded
        # Use a small tolerance (1e-6) for safe floating-point comparisons
        if self.current_position > self.max_position + 1e-6:
            self.get_logger().info('Maximum position reached. Stopping publishing.')
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

    gripper_commander_node = CheckGripper()

    try:
        rclpy.spin(gripper_commander_node)
    except KeyboardInterrupt:
        pass
    finally:
        gripper_commander_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
