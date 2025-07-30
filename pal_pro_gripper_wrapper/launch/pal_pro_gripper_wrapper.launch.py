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

import os
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory
from launch_pal.param_utils import parse_parametric_yaml
from launch_pal.arg_utils import LaunchArgumentsBase
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from tiago_pro_description.launch_arguments import TiagoProArgs


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    arm_type_right: DeclareLaunchArgument = TiagoProArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoProArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoProArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoProArgs.end_effector_left

    side: DeclareLaunchArgument = DeclareLaunchArgument(
        name='side',
        default_value='',
        description='side of the end effector, can be left empty')


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    # Add controller of left gripper
    launch_description.add_action(OpaqueFunction(
        function=set_side_gripper, args=['left'],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('end_effector_left'), "' != 'no-end-effector' and '",
                 LaunchConfiguration('arm_type_left'), "' != 'no-arm'"]
            )
        )))

    # Add controller of right gripper
    launch_description.add_action(OpaqueFunction(
        function=set_side_gripper, args=['right'],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('end_effector_right'), "' != 'no-end-effector' and '",
                 LaunchConfiguration('arm_type_right'), "' != 'no-arm'"]
            )
        )))

    return


def set_side_gripper(context, side='', *args, **kwargs):

    ee_prefix = "gripper"
    if side:
        ee_prefix = f"gripper_{side}"

    remappings = {"EE_SIDE_PREFIX": ee_prefix}
    grasp_check_params = os.path.join(
        get_package_share_directory('pal_pro_gripper_wrapper'),
        'config', 'gripper.yaml')

    parsed_yaml = parse_parametric_yaml(source_files=[grasp_check_params],
                                        param_rewrites=remappings)

    grasp_check_srv = Node(
        package='pal_pro_gripper_wrapper',
        name=f'{ee_prefix}_grasper_srv',
        executable='gripper_grasper_srv',
        output='screen',
        emulate_tty=True,
        parameters=[parsed_yaml],
    )

    return [grasp_check_srv]


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
