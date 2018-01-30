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

#include <gtest/gtest.h>

#include <xpp_vis/rviz_robot_builder.h>

using namespace xpp;

TEST(RvizRobotBuilder, BuildRobotState)
{
  RvizRobotBuilder builder;

  // defines the robot state
  xpp_msgs::RobotParameters params_msg;
  params_msg.base_mass = 30.0; // [kg]
  params_msg.ee_max_dev.x = params_msg.ee_max_dev.x = params_msg.ee_max_dev.z = 0.1;
  builder.SetRobotParameters(params_msg);

  // create a biped robot standing 0.6m tall
  xpp_msgs::RobotStateCartesian state_msg;
  state_msg.ee_motion.resize(2);
  state_msg.ee_forces.resize(2);
  state_msg.base.pose.position.z = 0.6; // [m]
  state_msg.ee_contact = { true, true };
  state_msg.ee_motion.at(0).pos.y =  0.2; // leg leg
  state_msg.ee_motion.at(1).pos.y = -0.2; // right leg
  state_msg.ee_forces.at(0).z = params_msg.base_mass*9.81/2.; // leg leg
  state_msg.ee_forces.at(1).z = params_msg.base_mass*9.81/2.; // right leg

  auto rviz_markers = builder.BuildRobotState(state_msg);

  // mostly checking for segfaults here
  EXPECT_FALSE(rviz_markers.markers.empty());
}
