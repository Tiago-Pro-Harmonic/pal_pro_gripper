cmake_minimum_required(VERSION 2.8.3)
project(pal_pro_gripper_wrapper)


find_package(catkin REQUIRED)

catkin_package(
)


#############
## Install ##
#############

install(PROGRAMS
  scripts/gripper_grasping.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach (dir launch config)
  install(DIRECTORY ${dir}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


