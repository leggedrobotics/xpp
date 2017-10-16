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
#include <xpp_vis/urdf_visualizer.h>

#include <xpp_vis_hyq/hyq_inverse_kinematics.h>
#include <xpp_vis_hyq/joints_monoped.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "hyq_urdf_visualizer");


	// urdf joint names
	int n_ee = quad::kMapIDToEE.size();
	int n_j  = KNumJointsMono;
	std::vector<std::string> joint_names(n_ee*n_j);
	joint_names.at(n_j*kMapIDToEE.at(LF) + HAA) = "lf_haa_joint";
	joint_names.at(n_j*kMapIDToEE.at(LF) + HFE) = "lf_hfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(LF) + KFE) = "lf_kfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(RF) + HAA) = "rf_haa_joint";
	joint_names.at(n_j*kMapIDToEE.at(RF) + HFE) = "rf_hfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(RF) + KFE) = "rf_kfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(LH) + HAA) = "lh_haa_joint";
	joint_names.at(n_j*kMapIDToEE.at(LH) + HFE) = "lh_hfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(LH) + KFE) = "lh_kfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(RH) + HAA) = "rh_haa_joint";
	joint_names.at(n_j*kMapIDToEE.at(RH) + HFE) = "rh_hfe_joint";
	joint_names.at(n_j*kMapIDToEE.at(RH) + KFE) = "rh_kfe_joint";


	auto hyq_ik = std::make_shared<HyqInverseKinematics>();
	CartesianJointConverter inv_kin_converter(hyq_ik,
	                                          xpp_msgs::robot_state_desired,
	                                          xpp_msgs::joint_desired);

	std::string urdf = "hyq_rviz_urdf_robot_description";
	UrdfVisualizer hyq_desired(joint_names, "base", urdf, "world",
	                           xpp_msgs::joint_desired, "hyq_des");


//  UrdfVisualizer hyq_current(hyq_ik, kMapXppJointToUrdfNames, urdf, "world",
//                             xpp_msgs::robot_state_current, "hyq_curr");

	std::cout<<"Created hyq_urdf_visualizer"<<std::endl;

	::ros::spin();

	return 1;
}

