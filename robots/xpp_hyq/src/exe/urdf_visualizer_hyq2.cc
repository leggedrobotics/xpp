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

#include <xpp_hyq/inverse_kinematics_hyq2.h>
#include <xpp_msgs/topic_names.h>

#include <xpp_vis/urdf_visualizer.h>
#include <xpp_vis/cartesian_joint_converter.h>

#include <xpp_states/endeffector_mappings.h>

using namespace xpp;
using namespace biped;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "biped_urdf_visualizer");

  const std::string joint_desired_biped = "xpp/joint_biped_des";

  auto ik = std::make_shared<InverseKinematicsHyq2>();
  CartesianJointConverter inv_kin_converter(ik,
					    xpp_msgs::robot_state_desired,
					    joint_desired_biped);

  int n_ee = ik->GetEECount();
  int n_j  = HyqlegJointCount;
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
  joint_names.at(n_j*L + HAA) = "L_haa_joint";
  joint_names.at(n_j*L + HFE) = "L_hfe_joint";
  joint_names.at(n_j*L + KFE) = "L_kfe_joint";
  joint_names.at(n_j*R + HAA) = "R_haa_joint";
  joint_names.at(n_j*R + HFE) = "R_hfe_joint";
  joint_names.at(n_j*R + KFE) = "R_kfe_joint";

  std::string urdf = "biped_rviz_urdf_robot_description";
  UrdfVisualizer node(urdf, joint_names, "base", "world",
		      joint_desired_biped, "biped");

  ::ros::spin();

  return 1;
}

