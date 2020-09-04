cmake_minimum_required(VERSION 3.0)
project(dmp_deformations)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)


find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib tf2_ros geometry_msgs sensor_msgs std_msgs genmsg relaxed_ik panda_ros_msgs)
catkin_package()
find_package(Eigen3 REQUIRED)
find_library(DHDC libdhd.so.3.8.0 REQUIRED)
find_library(DRDC libdrd.so.3.8.0 REQUIRED)
find_library(LIBNIFALCON nifalcon REQUIRED)
find_library(franka libfranka.so.0.7.1 REQUIRED)
find_library(
    pandaController
    NAMES PandaController
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)
find_library(BSpline libBSpline.so REQUIRED)
find_library(NLopt libnlopt.so REQUIRED)

message(STATUS ${pandaController})
find_library(
    pandaCommon
    NAMES common 
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)

add_executable(fddeformationController src/ForceDimensionDeformationController.cpp)
target_include_directories(fddeformationController PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(fddeformationController ${catkin_LIBRARIES} ${DHDC} ${BSpline} ${NLopt})

add_executable(falcondeformationController src/FalconDeformationController.cpp)
target_include_directories(falcondeformationController PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(falcondeformationController ${catkin_LIBRARIES} ${LIBNIFALCON} ${DHDC} ${BSpline} ${NLopt})

add_executable(fdbilateral src/ForceDimension_Bilateral.cpp)
target_include_directories(fdbilateral PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(fdbilateral ${catkin_LIBRARIES} ${DHDC})

add_executable(falconbilateral src/Falcon_Bilateral.cpp)
target_include_directories(falconbilateral PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(falconbilateral ${catkin_LIBRARIES} ${LIBNIFALCON} ${DHDC})

add_executable(falconVF src/Falcon_VF_surface.cpp)
target_include_directories(falconVF PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(falconVF ${catkin_LIBRARIES} ${LIBNIFALCON} ${DHDC} ${BSpline} ${NLopt})

add_executable(fdVF src/ForceDimension_surfacedivisible.cpp)
target_include_directories(fdVF PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(fdVF ${catkin_LIBRARIES} ${LIBNIFALCON} ${DHDC} ${BSpline} ${NLopt})

install(TARGETS fddeformationController
  RUNTIME DESTINATION lib/fddeformations
)

install(TARGETS falcondeformationController
  RUNTIME DESTINATION lib/falcondeformations
)

install(TARGETS fdbilateral
  RUNTIME DESTINATION lib/fdbilateral
)

install(TARGETS falconbilateral
  RUNTIME DESTINATION lib/falconbilateral
)

install(TARGETS falconVF
  RUNTIME DESTINATION lib/falconVF
)

install(TARGETS fdVF
  RUNTIME DESTINATION lib/fdVF
)