# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_laser_snapshotter)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs std_msgs pr2_msgs laser_assembler message_generation message_runtime actionlib actionlib_msgs)

add_service_files(DIRECTORY srv FILES BuildCloudAngle.srv)

add_action_files(DIRECTORY action 
FILES TiltLaserSnapshot.action
)

include_directories(include ${catkin_INCLUDE_DIRS})

generate_messages(
DEPENDENCIES
std_msgs actionlib_msgs sensor_msgs geometry_msgs pr2_msgs)

add_executable(pr2_laser_snapshotter_action src/pr2_laser_snapshotter_action.cpp)
target_link_libraries(pr2_laser_snapshotter_action ${CATKIN_LIBRARIES})

add_executable(pr2_laser_snapshotter src/pr2_laser_snapshotter.cpp)
target_link_libraries(pr2_laser_snapshotter ${CATKIN_LIBRARIES})


catkin_package(
    DEPENDS roscpp sensor_msgs geometry_msgs pr2_msgs laser_assembler actionlib actionlib_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(TARGETS pr2_laser_snapshotter_action pr2_laser_snapshotter
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY action src srv test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

