cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)

rosbuild_init()

rosbuild_find_ros_package(ros_epic_fail)
include(${ros_epic_fail_PACKAGE_PATH}/run_epic_fail.cmake)
epic_fail_labeled(gazebo)

rosbuild_add_rostest_labeled(gazebo launch/pr2_doors_test.launch)
