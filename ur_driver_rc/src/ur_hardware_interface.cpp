/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface that performs a perfect control loop for simulation
*/

#include <ur_driver_rc/ur_hardware_interface.h>

namespace ur_driver_rc
{

URHardwareInterface::URHardwareInterface(ros::NodeHandle& nh)
  : nh_(nh)
{
  // Initialize shared memory and interfaces here
  init(); // this implementation loads from rosparam

  ROS_INFO_NAMED("ur_hardware_interface", "Loaded ur_hardware_interface.");
}

void URHardwareInterface::init()
{
  ROS_INFO_STREAM_NAMED("ur_hardware_interface","Establish connection to UR");
  
  std::string host = std::string("192.168.1.42");
  int port = 30002;

  arm_ = new Arm(host, port);
  
  arm_->init();
  
  
  
  
  ROS_INFO_STREAM_NAMED("ur_hardware_interface","Reading rosparams from namespace: " << nh_.getNamespace());

  // Get joint names
/*
  nh_.getParam("hardware_interface/joints", joint_names_);
  if (joint_names_.size() == 0)
  {
    ROS_FATAL_STREAM_NAMED("ur_hardware_interface","No joints found on parameter server for controller, did you load the proper yaml file?"
                           << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }
*/
  
  joint_names_.push_back("shoulder_pan_joint");
  joint_names_.push_back("shoulder_lift_joint");
  joint_names_.push_back("elbow_joint");
  joint_names_.push_back("wrist_1_joint");
  joint_names_.push_back("wrist_2_joint");
  joint_names_.push_back("wrist_3_joint");
  
  num_joints_ = joint_names_.size();

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  //joint_position_command_.resize(num_joints_);
  //joint_velocity_command_.resize(num_joints_);
  //joint_effort_command_.resize(num_joints_);

  // Initialize controller
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    ROS_DEBUG_STREAM_NAMED("ur_hardware_interface","Loading joint name: " << joint_names_[i]);

    // Create joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

    //// Create position joint interface
    //position_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),&joint_position_command_[i]));
  }
  registerInterface(&joint_state_interface_); // From RobotHW base class.
  //registerInterface(&position_joint_interface_); // From RobotHW base class.
  //registerInterface(&velocity_joint_interface_); // From RobotHW base class.
  //registerInterface(&effort_joint_interface_); // From RobotHW base class.
}

void URHardwareInterface::read(ros::Duration elapsed_time)
{
  arm_->update();
  
  // Read the joint states from your hardware here
  JointAngles angles = arm_->getJointAngles();
  JointSpeeds speeds = arm_->getJointSpeeds();
  
  joint_position_[0] = angles.base;
  joint_velocity_[0] = speeds.base;
  joint_effort_[0] = 0.0;
  
  joint_position_[1] = angles.shoulder;
  joint_velocity_[1] = speeds.shoulder;
  joint_effort_[1] = 0.0;
  
  joint_position_[2] = angles.elbow;
  joint_velocity_[2] = speeds.elbow;
  joint_effort_[2] = 0.0;
  
  joint_position_[3] = angles.wrist1;
  joint_velocity_[3] = speeds.wrist1;
  joint_effort_[3] = 0.0;
  
  joint_position_[4] = angles.wrist2;
  joint_velocity_[4] = speeds.wrist2;
  joint_effort_[4] = 0.0;
  
  joint_position_[5] = angles.wrist2;
  joint_velocity_[5] = speeds.wrist3;
  joint_effort_[5] = 0.0;
}

void URHardwareInterface::write(ros::Duration elapsed_time)
{
  //empty for now
}


} // namespace
