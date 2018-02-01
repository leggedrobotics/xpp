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

#include <ros/ros.h>

#include <xpp_vis/urdf_visualizer.h>

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/topic_names.h>

using namespace xpp;

ros::Publisher joint_state_pub;

// convert cartesian to joint state message (just by extracting base)
void StateCallback(const xpp_msgs::RobotStateCartesian& msg)
{
  xpp_msgs::RobotStateJoint joint_msg;
  joint_msg.base = msg.base;
  joint_state_pub.publish(joint_msg);
}


int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "quadrotor_urdf_visualizer");

  ::ros::NodeHandle n;
  const std::string joint_quadrotor = "xpp/joint_quadrotor_des";
  ros::Subscriber cart_state_sub = n.subscribe(xpp_msgs::robot_state_desired, 1, StateCallback);
  joint_state_pub = n.advertise<xpp_msgs::RobotStateJoint>(joint_quadrotor, 1);

  // publish base state to RVIZ
  std::string urdf = "quadrotor_rviz_urdf_robot_description";
  UrdfVisualizer node_des(urdf, {}, "base", "world", joint_quadrotor, "quadrotor");

  ::ros::spin();

  return 0;
}

