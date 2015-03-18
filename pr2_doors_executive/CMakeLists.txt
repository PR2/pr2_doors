# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_doors_executive)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS pr2_doors_actions door_handle_detector pr2_navigation_teleop pr2_navigation_perception pr2_navigation_local pr2_controller_manager pr2_mechanism_msgs smach smach_ros)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS pr2_doors_actions door_handle_detector pr2_navigation_teleop pr2_navigation_perception pr2_navigation_local pr2_controller_manager pr2_mechanism_msgs smach smach_ros
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
