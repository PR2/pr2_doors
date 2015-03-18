# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_doors_actions)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp tf actionlib pr2_common_action_msgs pr2_arm_move_ik joint_trajectory_generator std_msgs diagnostic_msgs diagnostic_updater door_msgs move_base_msgs geometry_msgs actionlib_msgs pr2_controllers_msgs door_handle_detector pr2_doors_common tff_controller costmap_2d base_local_planner laser_assembler pr2_laser_snapshotter)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp tf actionlib pr2_common_action_msgs pr2_arm_move_ik joint_trajectory_generator std_msgs diagnostic_msgs diagnostic_updater door_msgs move_base_msgs geometry_msgs actionlib_msgs pr2_controllers_msgs door_handle_detector pr2_doors_common tff_controller costmap_2d base_local_planner laser_assembler pr2_laser_snapshotter
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
