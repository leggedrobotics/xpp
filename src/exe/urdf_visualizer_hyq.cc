/**
@file    hyqb_vis_node.cpp
@author  Diego Pardo (depardo@ethz.ch) & Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2015
@brief   
 */

#include <ros/ros.h>

#include <xpp/quad/hyq_inverse_kinematics.h>
#include <xpp/quad/joints_quadruped.h>
#include <xpp/urdf_visualizer.h>
#include <xpp/ros/topic_names.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "hyq_urdf_visualizer");

	auto map = GetMap(kMapQuadEEToJoints);

	// this is how the joints are called in the URDF file
	std::map<xpp::JointID, std::string> kMapXppJointToUrdfNames {
	  { BaseJoint,               "base" },
    { map.at(LF_HAA),  "lf_haa_joint" },
    { map.at(LF_HFE),  "lf_hfe_joint" },
    { map.at(LF_KFE),  "lf_kfe_joint" },
    { map.at(RF_HAA),  "rf_haa_joint" },
    { map.at(RF_HFE),  "rf_hfe_joint" },
    { map.at(RF_KFE),  "rf_kfe_joint" },
    { map.at(LH_HAA),  "lh_haa_joint" },
    { map.at(LH_HFE),  "lh_hfe_joint" },
    { map.at(LH_KFE),  "lh_kfe_joint" },
    { map.at(RH_HAA),  "rh_haa_joint" },
    { map.at(RH_HFE),  "rh_hfe_joint" },
    { map.at(RH_KFE),  "rh_kfe_joint" }
	};

	auto hyq_ik = std::make_shared<HyqInverseKinematics>();
	std::string urdf = "hyq_rviz_urdf_robot_description";

	UrdfVisualizer hyq_desired(hyq_ik, kMapXppJointToUrdfNames, urdf, "world",
	                           xpp_msgs::robot_state_desired, "hyq");

//  UrdfVisualizer hyq_current(hyq_ik, kMapXppJointToUrdfNames, urdf, "world",
//                             xpp_msgs::robot_state_current, "hyq_curr");


	std::cout<<"Created hyq_urdf_visualizer"<<std::endl;

	::ros::spin();

	return 1;
}

