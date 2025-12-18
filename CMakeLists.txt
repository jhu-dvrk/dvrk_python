cmake_minimum_required (VERSION 3.16)
project (dvrk_python VERSION 2.4.0)

# first test for ROS1
find_package (catkin QUIET
  COMPONENTS rospy)

if (catkin_FOUND)

  catkin_package (CATKIN_DEPENDS rospy)
  catkin_python_setup ()

else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    find_package (ament_cmake_python REQUIRED)
    ament_python_install_package (dvrk
      PACKAGE_DIR ${dvrk_python_SOURCE_DIR}/src/dvrk)

    file (GLOB dvrk_SCRIPTS scripts/*.py)
    install (PROGRAMS ${dvrk_SCRIPTS}
      DESTINATION lib/${PROJECT_NAME})

    ament_package ()
  endif (ament_cmake_FOUND)

endif (catkin_FOUND)
