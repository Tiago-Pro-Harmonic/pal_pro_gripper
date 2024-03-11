^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_pro_gripper_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2024-03-11)
------------------
* Merge branch 'dtk/fix/mimic-join-hack' into 'humble-devel'
  Dtk/fix/mimic join hack
  See merge request robots/pal_pro_gripper!10
* Remove commented lines
* Add dummy link for the mimic joints
* Slight refactor of mimic joint ros2 control
* Merge branch 'dtk/fix/add-linter-tests' into 'humble-devel'
  Dtk/fix/add linter tests
  See merge request robots/pal_pro_gripper!11
* Add tests packages to package.xml
* Add linter tests in CMakeLists.txt
* Contributors: David ter Kuile, davidterkuile

1.0.2 (2024-03-06)
------------------

1.0.1 (2024-01-31)
------------------
* Merge branch 'fix-collision' into 'humble-devel'
  Fix collision
  See merge request robots/pal_pro_gripper!8
* simplify the root link adding a world link
* fix collision of the little box support
* Contributors: Adria Roig, ileniaperrella

1.0.0 (2024-01-29)
------------------
* Merge branch 'fix-spawn' into 'humble-devel'
  add world_link to improve the spawn in simulation
  See merge request robots/pal_pro_gripper!7
* add world_link to improve the spawn in simulation
* Merge branch 'ros2-migration' into 'humble-devel'
  Ros2 migration
  See merge request robots/pal_pro_gripper!5
* add working mimic joint configuration
* box support for the gripper reduced
* fix typo
* clean gazebo.urdf.xacro adding gazebo_ros2_control
* pal_gazebo_worlds exc_depend added
* update to 3.8 the cmake_minimum_required Version
* fix deg_to_rad extension
* add mimic joint for outer_finger_left_joint
* comment mimic joint gazebo plugin
* update show launch with rviz_config file
* working version of the pal_pro_gripper
* clean conf files pal_pro_gripper_description
* gripper transmission file modified
* fix name error in the ros2_control.xacro
* delete not necessary dependencies
* delete commented lines
* added robot_state_publisher to the gazebo spawn
* standalone gazebo launch files
* launch files for the spawn of the gripper in visualization
* pal_pro_gripper_description pkg migration files
* gazebo.urdf.xacro with ros2 plugin
* report files that have not changed
* migration of CMakeLists.txt and package.xml to ros2
* Contributors: Adria Roig, ileniaperrella

0.0.3 (2023-10-23)
------------------
* Merge branch 'feat/use_urdf_utils' into 'main'
  Feat/use urdf utils
  See merge request robots/pal_pro_gripper!4
* remove materials to make use of pal_urdf_utils package
* Contributors: Jordan Palacios, thomaspeyrucain

0.0.2 (2023-07-11)
------------------

0.0.1 (2023-07-03)
------------------
* Update dependencies and tests
* Remove deprecated xacro --inorder from tests
* Merge branch 'create-urdf' into 'main'
  Create urdf
  See merge request robots/pal_pro_gripper!2
* Update joint limit after confirming with mechanics
* update joint limits
* Merge branch 'create-urdf' into 'main'
  Create urdf
  See merge request robots/pal_pro_gripper!1
* Update mimic joint direction
* Update gazebo launch file and urdf to start simulation
* Load materials in general robot instead of urdf
* Add materials again, update joint limits:
* Update pid gains
* Update mimicjoint and typos
* first commit adding description and urdf
* first commit adding description and urdf
* Contributors: David ter Kuile, davidterkuile
