/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <xpp_vis/cartesian_joint_converter.h>

#include <ros/node_handle.h>

#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_states/convert.h>

namespace xpp {

CartesianJointConverter::CartesianJointConverter (const InverseKinematics::Ptr& ik,
                                                  const std::string& cart_topic,
                                                  const std::string& joint_topic)
{
  inverse_kinematics_ = ik;

  ::ros::NodeHandle n;
  cart_state_sub_ = n.subscribe(cart_topic, 1, &CartesianJointConverter::StateCallback, this);
  ROS_DEBUG("Subscribed to: %s", cart_state_sub_.getTopic().c_str());

  joint_state_pub_  = n.advertise<xpp_msgs::RobotStateJoint>(joint_topic, 1);
  ROS_DEBUG("Publishing to: %s", joint_state_pub_.getTopic().c_str());
}

void
CartesianJointConverter::StateCallback (const xpp_msgs::RobotStateCartesian& cart_msg)
{
  auto cart = Convert::ToXpp(cart_msg);

  // transform feet from world -> base frame
  Eigen::Matrix3d B_R_W = cart.base_.ang.q.normalized().toRotationMatrix().inverse();
  EndeffectorsPos ee_B(cart.ee_motion_.GetEECount());
  for (auto ee : ee_B.GetEEsOrdered())
    ee_B.at(ee) = B_R_W * (cart.ee_motion_.at(ee).p_ - cart.base_.lin.p_);

  Eigen::VectorXd q =  inverse_kinematics_->GetAllJointAngles(ee_B).ToVec();

  xpp_msgs::RobotStateJoint joint_msg;
  joint_msg.base            = cart_msg.base;
  joint_msg.ee_contact      = cart_msg.ee_contact;
  joint_msg.time_from_start = cart_msg.time_from_start;
  joint_msg.joint_state.position = std::vector<double>(q.data(), q.data()+q.size());
  // Attention: Not filling joint velocities or torques

  joint_state_pub_.publish(joint_msg);
}

} /* namespace xpp */
