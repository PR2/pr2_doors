/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Wim Meeussen
 */

#include "tff_controller/tff_controller.h"
#include <algorithm>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_DECLARE_CLASS(tff_controller, TFFController, controller::TFFController, pr2_controller_interface::Controller)

using namespace KDL;
using namespace ros;



namespace controller {

TFFController::TFFController()
: robot_state_(NULL),
  jnt_to_twist_solver_(NULL),
  mode_(6),
  value_(6),
  twist_to_wrench_(6),
  state_position_publisher_(NULL)
{}



TFFController::~TFFController()
{}



bool TFFController::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle& n)
{
  node_ = n;

  // get name of root and tip from the parameter server
  std::string root_name, tip_name;
  if (!node_.getParam("root_name", root_name)){
    ROS_ERROR("TFFController: No root name found on parameter server");
    return false;
  }
  if (!node_.getParam("tip_name", tip_name)){
    ROS_ERROR("TFFController: No tip name found on parameter server");
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!chain_.init(robot_state, root_name, tip_name))
    return false;
  chain_.toKDL(kdl_chain_);

  // create solvers
  jnt_to_twist_solver_.reset(new ChainFkSolverVel_recursive(kdl_chain_));
  jnt_posvel_.resize(kdl_chain_.getNrOfJoints());
  jnt_to_jac_solver_.reset(new ChainJntToJacSolver(kdl_chain_));
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());
  jnt_eff_.resize(kdl_chain_.getNrOfJoints());
  jacobian_.resize(kdl_chain_.getNrOfJoints());

  // twist to wrench
  double trans, rot;
  node_.param("twist_to_wrench_trans", trans, 0.0);
  for (unsigned int i=0; i<3; i++)
    twist_to_wrench_[i] = trans;
  node_.param("twist_to_wrench_rot", rot, 0.0);
  for (unsigned int i=3; i<6; i++)
    twist_to_wrench_[i] = rot;

  // get pid controllers
  control_toolbox::Pid pid_controller;
  if (!pid_controller.init(NodeHandle(node_, "vel_trans"))) return false;
  for (unsigned int i=0; i<3; i++)
    vel_pid_controller_.push_back(pid_controller);

  if (!pid_controller.init(NodeHandle(node_, "vel_rot"))) return false;
  for (unsigned int i=0; i<3; i++)
    vel_pid_controller_.push_back(pid_controller);

  if (!pid_controller.init(NodeHandle(node_, "pos_trans"))) return false;
  for (unsigned int i=0; i<3; i++)
    pos_pid_controller_.push_back(pid_controller);

  if (!pid_controller.init(NodeHandle(node_, "pos_rot"))) return false;
  for (unsigned int i=0; i<3; i++)
    pos_pid_controller_.push_back(pid_controller);

  // subscribe to tff commands
  sub_command_ = node_.subscribe<tff_controller::TaskFrameFormalism>("command", 1,
                                 &TFFController::command, this);

  // realtime publisher for control state
  state_position_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(node_, "state/position", 1));

  return true;
}



void TFFController::starting()
{
  // time
  last_time_ = robot_state_->getTime();

  // set initial modes and values
  for (unsigned int i=0; i<6; i++){
    mode_[i] = tff_controller::TaskFrameFormalism::FORCE;
    value_[i] = 0;
  }

  // reset pid controllers
  for (unsigned int i=0; i<6; i++){
    vel_pid_controller_[i].reset();
    pos_pid_controller_[i].reset();
  }

  // set initial position, twist
  FrameVel frame_twist;
  chain_.getVelocities(jnt_posvel_);
  jnt_to_twist_solver_->JntToCart(jnt_posvel_, frame_twist);
  pose_meas_old_ = frame_twist.value();
  position_ = Twist::Zero();

  // set desired wrench to 0
  wrench_desi_ = Wrench::Zero();

  loop_count_ = 0;
}


void TFFController::update()
{
  // get time
  ros::Time time = robot_state_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;

  // get joint positions
  chain_.getPositions(jnt_pos_);

  // get the joint velocities
  chain_.getVelocities(jnt_posvel_);

  // get the chain jacobian
  jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);

  // get cartesian twist and pose
  FrameVel frame_twist;
  jnt_to_twist_solver_->JntToCart(jnt_posvel_, frame_twist);
  pose_meas_ = frame_twist.value();
  twist_meas_ = pose_meas_.M.Inverse() * (frame_twist.deriv());

  // calculate the distance traveled along the axes of the tf
  position_ += pose_meas_.M.Inverse() * diff(pose_meas_old_, pose_meas_);
  pose_meas_old_ = pose_meas_;

  // calculate desired wrench
  wrench_desi_ = Wrench::Zero();
  for (unsigned int i=0; i<6; i++){
    if (mode_[i] == tff_controller::TaskFrameFormalism::FORCE)
      wrench_desi_[i] = value_[i];
    else if (mode_[i] ==  tff_controller::TaskFrameFormalism::VELOCITY)
      wrench_desi_[i] = twist_to_wrench_[i] * (value_[i] + vel_pid_controller_[i].updatePid(twist_meas_[i] - value_[i], dt));
    else if (mode_[i] == tff_controller::TaskFrameFormalism::POSITION)
      wrench_desi_[i] = twist_to_wrench_[i] * (pos_pid_controller_[i].updatePid(position_[i] - value_[i], dt));
  }

  // convert wrench to base reference frame
  wrench_desi_ = (pose_meas_.M * wrench_desi_);

  // convert the wrench into joint efforts
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++){
    jnt_eff_(i) = 0;
    for (unsigned int j=0; j<6; j++)
      jnt_eff_(i) += (jacobian_(j,i) * wrench_desi_(j));
  }

  // set effort to joints
  chain_.setEfforts(jnt_eff_);

  // publish state
  if (++loop_count_ % 100 == 0){
    if (state_position_publisher_){
      if (state_position_publisher_->trylock()){
        state_position_publisher_->msg_.linear.x = position_.vel(0);
        state_position_publisher_->msg_.linear.y = position_.vel(1);
        state_position_publisher_->msg_.linear.z = position_.vel(2);
        state_position_publisher_->msg_.angular.x = position_.rot(0);
	state_position_publisher_->msg_.angular.y = position_.rot(1);
        state_position_publisher_->msg_.angular.z = position_.rot(2);
        state_position_publisher_->unlockAndPublish();
      }
    }
  }

}


void TFFController::command(const tff_controller::TaskFrameFormalismConstPtr& tff_msg)
{
  mode_[0] = trunc(tff_msg->mode.linear.x);
  mode_[1] = trunc(tff_msg->mode.linear.y);
  mode_[2] = trunc(tff_msg->mode.linear.z);
  mode_[3] = trunc(tff_msg->mode.angular.x);
  mode_[4] = trunc(tff_msg->mode.angular.y);
  mode_[5] = trunc(tff_msg->mode.angular.z);

  value_[0] = tff_msg->value.linear.x;
  value_[1] = tff_msg->value.linear.y;
  value_[2] = tff_msg->value.linear.z;
  value_[3] =  tff_msg->value.angular.x;
  value_[4] =  tff_msg->value.angular.y;
  value_[5] =  tff_msg->value.angular.z;
}



}; // namespace
