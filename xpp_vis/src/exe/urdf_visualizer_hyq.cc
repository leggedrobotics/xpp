/**
@file    hyqb_vis_node.cpp
@author  Diego Pardo (depardo@ethz.ch) & Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2015
@brief   
 */

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/quad/hyq_inverse_kinematics.h>
//#include <xpp_vis/quad/joints_quadruped.h>
#include <xpp_vis/urdf_visualizer.h>

#include <xpp_vis/mono/joints_monoped.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "hyq_urdf_visualizer");

//	auto map = GetMap(kMapQuadEEToJoints);
//
//	// this is how the joints are called in the URDF file
//	std::map<xpp::JointID, std::string> kMapXppJointToUrdfNames {
//	  { BaseJoint,               "base" },
//    { map.at(LF_HAA),  "lf_haa_joint" },
//    { map.at(LF_HFE),  "lf_hfe_joint" },
//    { map.at(LF_KFE),  "lf_kfe_joint" },
//    { map.at(RF_HAA),  "rf_haa_joint" },
//    { map.at(RF_HFE),  "rf_hfe_joint" },
//    { map.at(RF_KFE),  "rf_kfe_joint" },
//    { map.at(LH_HAA),  "lh_haa_joint" },
//    { map.at(LH_HFE),  "lh_hfe_joint" },
//    { map.at(LH_KFE),  "lh_kfe_joint" },
//    { map.at(RH_HAA),  "rh_haa_joint" },
//    { map.at(RH_HFE),  "rh_hfe_joint" },
//    { map.at(RH_KFE),  "rh_kfe_joint" }
//	};





	// urdf joint names
	int n_ee = quad::kMapIDToEE.size();
	int n_j  = mono::KNumJoints;
	std::vector<std::string> joint_names(n_ee*n_j);
	joint_names.at(n_j*kMapIDToEE.at(LF) + mono::HAA) = "lf_haa_joint";
	joint_names.at(n_j*kMapIDToEE.at(LF) + mono::HFE) = "lf_hfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(LF) + mono::KFE) = "lf_kfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(RF) + mono::HAA) = "rf_haa_joint";
	joint_names.at(n_j*kMapIDToEE.at(RF) + mono::HFE) = "rf_hfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(RF) + mono::KFE) = "rf_kfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(LH) + mono::HAA) = "lh_haa_joint";
	joint_names.at(n_j*kMapIDToEE.at(LH) + mono::HFE) = "lh_hfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(LH) + mono::KFE) = "lh_kfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(RH) + mono::HAA) = "rh_haa_joint";
	joint_names.at(n_j*kMapIDToEE.at(RH) + mono::HFE) = "rh_hfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(RH) + mono::KFE) = "rh_kfe_joint";



	auto hyq_ik = std::make_shared<HyqInverseKinematics>();
	std::string urdf = "hyq_rviz_urdf_robot_description";

	CartesianJointConverter inv_kin_converter(hyq_ik,
	                                          xpp_msgs::robot_state_desired,
	                                          xpp_msgs::joint_desired);

	UrdfVisualizer hyq_desired(joint_names, "base", urdf, "world",
	                           xpp_msgs::joint_desired, "hyq_des");


//  UrdfVisualizer hyq_current(hyq_ik, kMapXppJointToUrdfNames, urdf, "world",
//                             xpp_msgs::robot_state_current, "hyq_curr");

	std::cout<<"Created hyq_urdf_visualizer"<<std::endl;

	::ros::spin();

	return 1;
}

