# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(door_handle_detector)
find_package(catkin REQUIRED COMPONENTS pr2_machine roscpp tf cv_bridge std_msgs geometry_msgs mapping_msgs door_msgs sensor_msgs stereo_msgs visualization_msgs std_srvs angles laser_assembler pr2_doors_common )

catkin_package(
    DEPENDS pr2_machine roscpp tf cv_bridge std_msgs geometry_msgs mapping_msgs door_msgs sensor_msgs stereo_msgs visualization_msgs std_srvs angles laser_assembler pr2_doors_common
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
