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
import numpy as np
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
        self.step = 0.05
        self.max_position = 0.95

        timer_period = 5.0  # secondi
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Nodo avviato. Pubblico una nuova posizione del gripper ogni 5 secondi.')

    def timer_callback(self):
        # Controlla se abbiamo superato la posizione massima
        # Usiamo una piccola tolleranza (1e-6) per confronti sicuri con i float
        if self.current_position > self.max_position + 1e-6:
            self.get_logger().info('Posizione massima raggiunta. Interruzione della pubblicazione.')
            self.timer.cancel()  # Ferma il timer
            return

        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['gripper_left_screw_joint']

        point = JointTrajectoryPoint()
        point.positions = [self.current_position]
        point.time_from_start = Duration(sec=1, nanosec=0)

        traj_msg.points.append(point)

        self.pal_gripper_pub.publish(traj_msg)
        self.get_logger().info(f'Pubblicata posizione gripper: {self.current_position:.2f}')

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
