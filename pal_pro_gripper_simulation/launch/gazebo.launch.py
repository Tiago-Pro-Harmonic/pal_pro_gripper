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
from os import environ, pathsep
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.arg_utils import LaunchArgumentsBase
from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    world_name: DeclareLaunchArgument = DeclareLaunchArgument(
        name='world_name',
        default_value='empty',
        description="Specify world name, we'll convert to full path")


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    set_sim_time = SetLaunchConfiguration('use_sim_time', 'True')
    launch_description.add_action(set_sim_time)

    robot_state_publisher = include_scoped_launch_py_description(
        pkg_name='pal_pro_gripper_description',
        paths=['launch', 'robot_state_publisher.launch.py'],
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')})

    launch_description.add_action(robot_state_publisher)

    packages = ['pal_pro_gripper_description']

    model_path = get_model_paths(packages)

    gazebo_model_path_env_var = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', model_path)

    gazebo = include_scoped_launch_py_description(
        pkg_name='pal_gazebo_worlds',
        paths=['launch', 'pal_gazebo.launch.py'],
        env_vars=[gazebo_model_path_env_var],
        launch_arguments={
            "world_name":  launch_args.world_name,
            "model_paths": packages,
            "resource_paths": packages,
        })

    launch_description.add_action(gazebo)

    pal_pro_gripper_spawn = include_scoped_launch_py_description(
        pkg_name='pal_pro_gripper_simulation', paths=[
            'launch', 'robot_spawn.launch.py'],
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')})

    launch_description.add_action(pal_pro_gripper_spawn)

    pal_pro_gripper_controller = include_scoped_launch_py_description(
        pkg_name='pal_pro_gripper_controller_configuration', paths=[
            'launch', 'pal_pro_gripper_controller_standalone.launch.py'],
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')})

    launch_description.add_action(pal_pro_gripper_controller)
    return


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def get_model_paths(packages_names):
    model_paths = ''
    for package_name in packages_names:
        if model_paths != '':
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, 'share')

        model_paths += model_path

    if 'GAZEBO_MODEL_PATH' in environ:
        model_paths += pathsep + environ['GAZEBO_MODEL_PATH']

    return model_paths


def get_resource_paths(packages_names):
    resource_paths = ''
    for package_name in packages_names:
        if resource_paths != '':
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_paths += pathsep + environ['GAZEBO_RESOURCE_PATH']

    return resource_paths
