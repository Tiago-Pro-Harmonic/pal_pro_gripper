# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_pal.include_utils import include_launch_py_description


def get_model_paths(packages_names):
    model_paths = ''
    for package_name in packages_names:
        if model_paths != '':
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, 'share')

        model_paths += model_path

    return model_paths


def get_resource_paths(packages_names):
    resource_paths = ''
    for package_name in packages_names:
        if resource_paths != '':
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    return resource_paths


def generate_launch_description():

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='empty',
        description="Specify world name, we'll convert to full path"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pal_gazebo_worlds'),
            'launch'), '/pal_gazebo.launch.py']),
    )

    pal_pro_gripper_spawn = include_launch_py_description(
        'pal_pro_gripper_description', [
            'launch', 'pal_pro_gripper_spawn.launch.py'],
        launch_arguments={'use_sim_time': 'True'}.items())

    packages = ['pal_pro_gripper_description']
    model_path = get_model_paths(packages)
    resource_path = get_resource_paths(packages)

    if 'GAZEBO_MODEL_PATH' in environ:
        model_path += pathsep + environ['GAZEBO_MODEL_PATH']

    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GAZEBO_RESOURCE_PATH']

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))
    # Using this prevents shared library from being found
    # ld.add_action(SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', tiago_resource_path))

    ld.add_action(world_name_arg)
    ld.add_action(gazebo)
    ld.add_action(pal_pro_gripper_spawn)

    return ld
