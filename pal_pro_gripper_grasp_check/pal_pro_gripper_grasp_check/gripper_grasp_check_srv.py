# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
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
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_srvs.srv import Empty
from std_msgs.msg import Bool


class GripperChecker(Node):
    def __init__(self) -> None:
        super().__init__('gripper_grasp_check_srv',
                         automatically_declare_parameters_from_overrides=True)

        # Init Params - defaults
        self.last_state = None
        self.controller_name = str(self.get_parameter("controller_name").value[0])
        self.real_joint_names = self.get_parameter("real_joint_names").value
        self.max_position_error = float(self.get_parameter('max_position_error').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.rate = float(self.get_parameter('rate').value)
        self.pub_rate = float(self.get_parameter('pub_rate').value)
        self.tolerance = float(self.get_parameter('tolerance').value)
        self.opening_time = float(self.get_parameter('opening_time').value)
        self.closing_time = float(self.get_parameter('closing_time').value)

        # Subs to gripper state
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState, f'/{self.controller_name}\
            /controller_state', self.state_cb, qos_profile=1)
        self.get_logger().info("Subscribed to topic: " + str(
            self.state_sub.topic_name))

        # Publisher on the gripper topic
        self.cmd_pub = self.create_publisher(
            JointTrajectory, f'/{self.controller_name}/joint_trajectory', 10)
        self.get_logger().info("Publishing on topic: " + str(
            self.cmd_pub.topic_name))

        # Graspng srv to offer
        self.grasp_srv = self.create_service(
            Empty, f'/{self.controller_name}/grasp', self.grasp_cb)
        self.get_logger().info("Offering grasp srv on: " + str(
            self.grasp_srv.srv_name))

        self.release_srv = self.create_service(
            Empty, f'/{self.controller_name}/release', self.open_cb)
        self.get_logger().info("Offering release srv on: " + str(
            self.release_srv.srv_name))

        # Publish a boolean to know if an object is grasped or not
        self.pub_grasp_state = self.create_publisher(Bool, 'is_grasped', 10)
        self.get_logger().info("Publishing on topic: " + str(
            self.pub_grasp_state.topic_name))

        self.is_grasped = Bool()
        self.on_optimal_close = False
        self.on_optimal_open = False

        self.get_logger().info("Initialized. Ready..")

    def state_cb(self, msg: JointTrajectoryControllerState) -> None:
        self.last_state = msg

        # Check if it's grasping or not
        if self.on_optimal_close:
            self.is_grasped.data = True
            if -self.last_state.error.positions[0] < self.tolerance:
                self.is_grasped.data = False
                self.on_optimal_close = False
        else:
            self.is_grasped.data = False

        # Publishing state
        self.pub_grasp_state.publish(self.is_grasped)

    def open_cb(self, req, res) -> Empty.Response:
        self.get_logger().debug("Recieved open request")
        # In any case we open the gripper
        opening_ammount = [0.0]

        # Handle the case if the srv is called again after a successfull grasp
        if not self.on_optimal_open:
            self.send_joint_traj(opening_ammount, self.opening_time)
            self.on_optimal_close = False
            self.get_clock().sleep_for(Duration(seconds=self.opening_time))

        self.get_logger().debug("Gripper opened!")
        return Empty.Response()

    def grasp_cb(self, req, res) -> Empty.Response:
        self.get_logger().debug("Recieved grasp request")

        # Keep closing the gripper until the error of the state reaches
        # max_position_error or any of the gripper joints (or timeout)
        init_time = self.get_clock().now()
        closing_ammount = [0.8]

        # Handle the case if the srv is called again after a successfull grasp
        if not self.on_optimal_close:
            self.get_logger().info("Closing: " + str(closing_ammount))
            self.send_joint_traj(closing_ammount, self.closing_time)
            self.on_optimal_open = False
            self.get_clock().sleep_for(Duration(seconds=self.closing_time))

        condition = rclpy.ok() and (
            self.get_clock().now() - init_time) < Duration(
                seconds=self.timeout) and not self.on_optimal_close

        while condition:

            if self.last_state is None:
                self.get_logger().warn("Waiting for gripper state...")
                continue

            current_error = self.last_state.error.positions[0]
            self.get_logger().info(f"Current error: {current_error}")

            if abs(current_error) > self.max_position_error:
                self.get_logger().debug("Over error joint 0..")
                closing_ammount = self.get_optimal_close()
                self.on_optimal_close = True

            self.get_logger().info("Closing: " + str(closing_ammount))
            self.send_joint_traj(closing_ammount, self.closing_time)
            self.get_clock().sleep_for(Duration(seconds=self.closing_time))

        self.get_logger().debug("Gripper closed!")
        return Empty.Response()

    def get_optimal_close(self) -> list[float]:
        optimal_0 = self.last_state.actual.positions[0] - self.max_position_error
        self.get_logger().info(f"Optimal close: {optimal_0}")
        return [optimal_0]

    def send_joint_traj(self, j_positions: list[float], exec_time: float) -> None:
        jt = JointTrajectory()
        jt.joint_names = self.real_joint_names
        p = JointTrajectoryPoint()
        p.positions = j_positions
        p.time_from_start = Duration(seconds=exec_time).to_msg()
        jt.points.append(p)

        self.get_logger().info("Sending traj" + str(jt))
        self.cmd_pub.publish(jt)
        return


def main(args=None):
    rclpy.init()
    gc = GripperChecker()
    rclpy.spin(gc)


if __name__ == '__main__':
    main()
