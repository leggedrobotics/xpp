/**
@file    hyqb_vis_node.cpp
@author  Diego Pardo (depardo@ethz.ch) & Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2015
@brief   
 */

#include <ros/ros.h>

#include <xpp/urdf_visualizer.h>
#include <xpp/hyq/joints_hyq.h>
#include <xpp/hyq/hyq_inverse_kinematics.h>

using namespace xpp;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "hyq_urdf_visualizer");

	// this is how the joints are called in the URDF file
	std::map<xpp::JointID, std::string> kMapXppJointToUrdfNames {
    { hyq::kMapHyqToXpp.at(hyq::LF_HAA),  "lf_haa_joint" },
    { hyq::kMapHyqToXpp.at(hyq::LF_HFE),  "lf_hfe_joint" },
    { hyq::kMapHyqToXpp.at(hyq::LF_KFE),  "lf_kfe_joint" },
    { hyq::kMapHyqToXpp.at(hyq::RF_HAA),  "rf_haa_joint" },
    { hyq::kMapHyqToXpp.at(hyq::RF_HFE),  "rf_hfe_joint" },
    { hyq::kMapHyqToXpp.at(hyq::RF_KFE),  "rf_kfe_joint" },
    { hyq::kMapHyqToXpp.at(hyq::LH_HAA),  "lh_haa_joint" },
    { hyq::kMapHyqToXpp.at(hyq::LH_HFE),  "lh_hfe_joint" },
    { hyq::kMapHyqToXpp.at(hyq::LH_KFE),  "lh_kfe_joint" },
    { hyq::kMapHyqToXpp.at(hyq::RH_HAA),  "rh_haa_joint" },
    { hyq::kMapHyqToXpp.at(hyq::RH_HFE),  "rh_hfe_joint" },
    { hyq::kMapHyqToXpp.at(hyq::RH_KFE),  "rh_kfe_joint" }
	};

	auto hyq_ik = std::make_shared<hyq::HyqInverseKinematics>();
	std::string urdf = "hyq_rviz_urdf_robot_description";

	UrdfVisualizer node(hyq_ik, kMapXppJointToUrdfNames, urdf);
	std::cout<<"Created hyq_urdf_visualizer"<<std::endl;

	::ros::spin();

	return 1;
}

