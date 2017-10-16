/**
@file    urdf_visualizer_biped.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2017
 */

#include <ros/ros.h>

#include <xpp_msgs/topic_names.h>

#include <xpp_vis/urdf_visualizer.h>
#include <xpp_vis/cartesian_joint_converter.h>

#include <xpp_vis_hyq/biped_inverse_kinematics.h>
#include <xpp_vis_hyq/joints_monoped.h>


using namespace xpp;
using namespace biped;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "biped_urdf_visualizer");

  int n_ee = biped::kMapIDToEE.size();
  int n_j  = KNumJointsMono;
  std::vector<std::string> joint_names(n_ee*n_j);
  joint_names.at(n_j*kMapIDToEE.at(L) + HAA) = "L_haa_joint";
  joint_names.at(n_j*kMapIDToEE.at(L) + HFE) = "L_hfe_joint";
  joint_names.at(n_j*kMapIDToEE.at(L) + KFE) = "L_kfe_joint";
  joint_names.at(n_j*kMapIDToEE.at(R) + HAA) = "R_haa_joint";
  joint_names.at(n_j*kMapIDToEE.at(R) + HFE) = "R_hfe_joint";
  joint_names.at(n_j*kMapIDToEE.at(R) + KFE) = "R_kfe_joint";


	auto ik = std::make_shared<BipedInverseKinematics>();
  CartesianJointConverter inv_kin_converter(ik,
                                            xpp_msgs::robot_state_desired,
                                            xpp_msgs::joint_desired);


	std::string urdf = "biped_rviz_urdf_robot_description";
	UrdfVisualizer node(joint_names, "base", urdf, "world",
	                    xpp_msgs::joint_desired, "biped");

//  UrdfVisualizer node_biped(ik, kMapXppJointToUrdfNames, urdf, "world",
//                      "xpp/state_biped", "biped");

	std::cout<<"Created biped_urdf_visualizer"<<std::endl;

	::ros::spin();

	return 1;
}

