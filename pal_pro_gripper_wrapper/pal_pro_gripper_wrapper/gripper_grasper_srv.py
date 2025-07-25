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
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from rclpy.callback_groups import ReentrantCallbackGroup


class GripperGrasper(Node):
    def __init__(self) -> None:
        super().__init__('gripper_grasper_srv',
                         automatically_declare_parameters_from_overrides=True)

        # Init Params and Subscriptions - defaults
        self.init_params()
        self.init_subscriptions()

        self.get_logger().info("Initialized. Ready..")

    def init_params(self) -> None:
        self.last_state = None
        self.controller_name = self.get_parameter(
            'controller_name').get_parameter_value().string_value
        self.real_joint_names = self.get_parameter(
            'real_joint_names').get_parameter_value().string_array_value
        self.max_position_error = self.get_parameter(
            'max_position_error').get_parameter_value().double_value
        self.timeout = self.get_parameter(
            'timeout').get_parameter_value().double_value
        self.tolerance = self.get_parameter(
            'tolerance').get_parameter_value().double_value
        self.opening_time = self.get_parameter(
            'opening_time').get_parameter_value().double_value
        self.closing_time = self.get_parameter(
            'closing_time').get_parameter_value().double_value
        self.open_value = self.get_parameter(
            'open_value').get_parameter_value().double_array_value
        self.close_value = self.get_parameter(
            'close_value').get_parameter_value().double_array_value
        self.is_grasped = Bool()

        # Define if the gripper grasping without stressing the joint
        # applying the 'optimal_close' joint value
        self.has_grasped_object = False
        # True when it's open, False otherwise
        self.is_open = False

    def init_subscriptions(self) -> None:

        # Used to let the cb in the group to run concurrently
        self.cb_group = ReentrantCallbackGroup()

        # Subs to gripper state
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState, f'/{self.controller_name}/controller_state',
            self.state_cb, qos_profile=1, callback_group=self.cb_group)
        self.get_logger().info("Subscribed to topic: " + str(
            self.state_sub.topic_name))
        # Publisher on the gripper topic
        self.cmd_pub = self.create_publisher(
            JointTrajectory, f'/{self.controller_name}/joint_trajectory', 10)
        self.get_logger().info("Publishing on topic: " + str(
            self.cmd_pub.topic_name))

        # Graspng srv to offer
        self.grasp_srv = self.create_service(
            Empty, f'/{self.controller_name}/grasp', self.grasp_cb, callback_group=self.cb_group)
        self.get_logger().info("Offering grasp srv on: " + str(
            self.grasp_srv.srv_name))

        # Releasing srv to offer
        self.release_srv = self.create_service(
            Empty, f'/{self.controller_name}/release', self.open_cb)
        self.get_logger().info("Offering release srv on: " + str(
            self.release_srv.srv_name))

        # Publish a boolean to know if an object is grasped or not
        self.pub_grasp_state = self.create_publisher(Bool, 'is_grasped', 10)
        self.get_logger().info("Publishing on topic: " + str(
            self.pub_grasp_state.topic_name))

    def state_cb(self, msg: JointTrajectoryControllerState) -> None:
        self.last_state = msg

        # Check if it's grasping or not
        if self.has_grasped_object:
            self.is_grasped.data = True
            if self.last_state.error.positions[0] > self.tolerance:
                self.is_grasped.data = False
                self.has_grasped_object = False
        else:
            self.is_grasped.data = False

        # Publishing state
        self.pub_grasp_state.publish(self.is_grasped)

    def open_cb(self, req: Empty.Request, res: Empty.Response) -> Empty.Response:
        self.get_logger().info("Recieved open request")

        # In any case we open the gripper
        # Handle the case if the srv is called again after a successfull grasp
        if not self.is_open:
            self.send_joint_traj(self.open_value, self.opening_time)
            self.get_clock().sleep_for(Duration(seconds=self.opening_time))
            self.has_grasped_object = False
            self.is_open = True

        self.get_logger().info("Gripper opened!\n")
        return res

    def grasp_cb(self, req: Empty.Request, res: Empty.Response) -> Empty.Response:
        self.get_logger().info("Recieved grasp request")

        # Keep closing the gripper until the error of the state reaches
        # max_position_error or any of the gripper joints (or timeout)
        init_time = self.get_clock().now()
        close_val = self.close_value

        # Handle the case if the srv is called again after a successfull grasp
        if not self.has_grasped_object:
            self.send_joint_traj(close_val, self.closing_time)
            self.get_clock().sleep_for(Duration(seconds=self.closing_time))
            self.is_open = False

        while rclpy.ok() and (
            self.get_clock().now() - init_time) < Duration(
                seconds=self.timeout) and not self.has_grasped_object:

            if self.last_state is None:
                self.get_logger().warn("Waiting for gripper state...")
                continue

            current_error = self.last_state.error.positions[0]
            self.get_logger().info(f"Current abs error: {current_error} - T: {self.max_position_error}")

            if abs(current_error) > self.max_position_error:
                self.get_logger().info("Over error joint 0..")
                close_val = self.get_optimal_close()
                self.has_grasped_object = True
                self.send_joint_traj(close_val, self.closing_time)

            # self.get_clock().sleep_for(Duration(seconds=0.1))

        self.get_logger().info("Gripper closed!\n")
        return res

    # Get optimal value to close the gripper to not let the joint stress in case of grasp
    def get_optimal_close(self) -> list[float]:
        optimal_0 = self.last_state.feedback.positions[0] - self.last_state.error.positions[0]
        optimal_0 = min(max(self.open_value[0], optimal_0), self.close_value[0])
        self.get_logger().info(f"Optimal close: {optimal_0}")
        return [optimal_0]

    def send_joint_traj(self, j_positions: list[float], exec_time: float) -> None:
        jt = JointTrajectory()
        jt.joint_names = self.real_joint_names
        p = JointTrajectoryPoint()
        p.positions = j_positions
        p.time_from_start = Duration(seconds=exec_time).to_msg()
        jt.points.append(p)

        self.get_logger().info("Closing: " + str(j_positions[0]))
        self.cmd_pub.publish(jt)
        return


def main(args=None):
    rclpy.init()
    gg = GripperGrasper()
    executor = MultiThreadedExecutor()

    executor.add_node(gg)
    executor.spin()


if __name__ == '__main__':
    main()
