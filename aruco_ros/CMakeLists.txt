cmake_minimum_required(VERSION 2.8.3)
project(aruco_ros)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  roscpp
  rospy
  tf
  aruco
  aruco_msgs
  sensor_msgs
  visualization_msgs
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/ArucoThreshold.cfg
)

#en add
add_message_files(
  FILES
  custom_msgs.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

add_compile_options(-std=c++11)

#en change
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_ros_utils
  CATKIN_DEPENDS aruco sensor_msgs tf message_runtime 
)

include_directories(
  include)
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_library(aruco_ros_utils src/aruco_ros_utils.cpp)
target_link_libraries(aruco_ros_utils ${catkin_LIBRARIES})

#en add
add_executable(dynamic_correction src/dynamic_correction.cpp)
target_link_libraries(dynamic_correction ${catkin_LIBRARIES})

add_executable(single_correction src/simple_single_correction.cpp
                      src/aruco_ros_utils.cpp)
add_dependencies(single_correction ${PROJECT_NAME}_gencfg)
target_link_libraries(single_correction ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(single_central src/simple_single_central.cpp
                      src/aruco_ros_utils.cpp)
add_dependencies(single_central ${PROJECT_NAME}_gencfg)
target_link_libraries(single_central ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

install(TARGETS single double marker_publisher aruco_ros_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

foreach(dir etc launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

