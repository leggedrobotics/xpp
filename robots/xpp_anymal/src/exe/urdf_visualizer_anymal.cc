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

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_anymal/inverse_kinematics_anymal.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "anymal_urdf_visualizer");

  const std::string joint_desired_anymal = "xpp/joint_anymal_des";

  auto anymal_ik = std::make_shared<InverseKinematicsANYmal>();
  CartesianJointConverter inv_kin_converter(anymal_ik,
					    xpp_msgs::robot_state_desired,
					    joint_desired_anymal);

  // urdf joint names
  int n_ee = anymal_ik->GetEECount();
  int n_j  = ANYmallegJointCount;
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
  joint_names.at(n_j*LF + HAA) = "LF_HAA";
  joint_names.at(n_j*LF + HFE) = "LF_HFE";
  joint_names.at(n_j*LF + KFE) = "LF_KFE";
  joint_names.at(n_j*RF + HAA) = "RF_HAA";
  joint_names.at(n_j*RF + HFE) = "RF_HFE";
  joint_names.at(n_j*RF + KFE) = "RF_KFE";
  joint_names.at(n_j*LH + HAA) = "LH_HAA";
  joint_names.at(n_j*LH + HFE) = "LH_HFE";
  joint_names.at(n_j*LH + KFE) = "LH_KFE";
  joint_names.at(n_j*RH + HAA) = "RH_HAA";
  joint_names.at(n_j*RH + HFE) = "RH_HFE";
  joint_names.at(n_j*RH + KFE) = "RH_KFE";

  std::string urdf = "anymal_rviz_urdf_robot_description";
  UrdfVisualizer anymal_desired(urdf, joint_names, "base", "world",
			     joint_desired_anymal, "anymal_des");

  ::ros::spin();

  return 1;
}

