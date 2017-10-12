/**
@file    urdf_visualizer_biped.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2017
 */

#include <ros/ros.h>

#include <xpp/urdf_visualizer.h>
#include <xpp/biped/joints_biped.h>
#include <xpp/biped/biped_inverse_kinematics.h>

#include <xpp_msgs/topic_names.h>

using namespace xpp;
using namespace biped;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "biped_urdf_visualizer");

	auto map = GetMap(kMapBipedEEToJoints);

	// these strings must match the <joint name=...> tag in the URDF file
	// monoped_description/urdf/monoped.urdf
	std::map<xpp::JointID, std::string> kMapXppJointToUrdfNames {
	  { BaseJoint,             "base"},
    { map.at(L_HAA),  "L_haa_joint" },
    { map.at(L_HFE),  "L_hfe_joint" },
    { map.at(L_KFE),  "L_kfe_joint" },
    { map.at(R_HAA),  "R_haa_joint" },
    { map.at(R_HFE),  "R_hfe_joint" },
    { map.at(R_KFE),  "R_kfe_joint" },
	};

	auto ik = std::make_shared<BipedInverseKinematics>();
	std::string urdf = "biped_rviz_urdf_robot_description";

	UrdfVisualizer node(ik, kMapXppJointToUrdfNames, urdf, "world",
	                    xpp_msgs::robot_state_desired, "biped");

//  UrdfVisualizer node_biped(ik, kMapXppJointToUrdfNames, urdf, "world",
//                      "xpp/state_biped", "biped");

	std::cout<<"Created biped_urdf_visualizer"<<std::endl;

	::ros::spin();

	return 1;
}

