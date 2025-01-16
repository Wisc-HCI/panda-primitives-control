cmake_minimum_required(VERSION 3.0)
project(controller)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)
SET(CATKIN_PACKAGE_BIN_DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs genmsg )

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS
    nodes/mover nodes/controller_tester.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(${catkin_INCLUDE_DIRS})
catkin_package(CATKIN_DEPENDS )

#install(TARGETS falconRosWrapper
#  RUNTIME DESTINATION lib/falcon_ros
#)
