/**
@file    urdf_visualizer_anymal.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2017
@brief   
 */

#include <ros/ros.h>

#include <xpp/quad/hyq_inverse_kinematics.h>
#include <xpp/quad/joints_quadruped.h>
#include <xpp/urdf_visualizer.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "anymal_urdf_visualizer");

	auto map = GetMap(kMapQuadEEToJoints);

	// this is how the joints are called in the URDF file
	std::map<xpp::JointID, std::string> kMapXppJointToUrdfNames {
    { map.at(LF_HAA),  "LF_HAA" },
    { map.at(LF_HFE),  "LF_HFE" },
    { map.at(LF_KFE),  "LF_KFE" },
    { map.at(RF_HAA),  "RF_HAA" },
    { map.at(RF_HFE),  "RF_HFE" },
    { map.at(RF_KFE),  "RF_KFE" },
    { map.at(LH_HAA),  "LH_HAA" },
    { map.at(LH_HFE),  "LH_HFE" },
    { map.at(LH_KFE),  "LH_KFE" },
    { map.at(RH_HAA),  "RH_HAA" },
    { map.at(RH_HFE),  "RH_HFE" },
    { map.at(RH_KFE),  "RH_KFE" }
	};

	auto hyq_ik = std::make_shared<HyqInverseKinematics>();
	std::string urdf = "anymal_rviz_urdf_robot_description";

	UrdfVisualizer node(hyq_ik, kMapXppJointToUrdfNames, urdf);
	std::cout<<"Created anymal_urdf_visualizer"<<std::endl;

	::ros::spin();

	return 1;
}

