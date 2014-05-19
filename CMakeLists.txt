cmake_minimum_required(VERSION 2.8.3)
project(ros_aruco)

find_package(catkin REQUIRED message_generation std_msgs image_transport cv_bridge)

include_directories(${catkin_INCLUDE_DIRS})

SET(CMAKE_MODULE_PATH "/usr/lib/cmake")
find_package(aruco REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  Markers.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

add_executable(ros_aruco src/ros_aruco.cpp)
target_link_libraries(ros_aruco ${aruco_LIBS} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS 
    ros_aruco
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
