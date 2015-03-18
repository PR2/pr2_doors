# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_doors_gazebo_demo)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS pr2_doors_executive pr2_doors_actions actionlib pr2_gazebo pr2_controller_manager gazebo_ros rostopic pr2_tuckarm move_base_msgs)

# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS pr2_doors_executive pr2_doors_actions actionlib pr2_gazebo pr2_controller_manager gazebo_ros rostopic pr2_tuckarm  move_base_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
