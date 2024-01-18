cmake_minimum_required(VERSION 3.8)
project(pal_pro_gripper_description)

find_package(ament_cmake_auto REQUIRED)
find_package(xacro REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE config gazebo robots launch meshes urdf ros2_control)
