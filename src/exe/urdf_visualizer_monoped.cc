/**
@file    hyqb_vis_node.cpp
@author  Diego Pardo (depardo@ethz.ch) & Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2015
@brief   
 */

#include <ros/ros.h>

#include <xpp/urdf_visualizer.h>
#include <xpp/mono/joints_monoped.h>
#include <xpp/mono/monoped_inverse_kinematics.h>

using namespace xpp;
using namespace mono;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "monoped_urdf_visualizer");

	// these strings must match the <joint name=...> tag in the URDF file
	// monoped_description/urdf/monoped.urdf
	std::map<xpp::JointID, std::string> kMapXppJointToUrdfNames {
    { kMapMonoToXpp.at(HAA),  "haa_joint" },
    { kMapMonoToXpp.at(HFE),  "hfe_joint" },
    { kMapMonoToXpp.at(KFE),  "kfe_joint" },
	};

	auto ik = std::make_shared<MonopedInverseKinematics>();
	std::string urdf = "monoped_rviz_urdf_robot_description";

	UrdfVisualizer node(ik, kMapXppJointToUrdfNames, urdf);
	std::cout<<"Created monoped_urdf_visualizer"<<std::endl;

	::ros::spin();

	return 1;
}

