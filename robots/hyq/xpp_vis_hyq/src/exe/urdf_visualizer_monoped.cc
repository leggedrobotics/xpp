/**
@file    hyqb_vis_node.cpp
@author  Diego Pardo (depardo@ethz.ch) & Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2015
@brief   
 */

#include <ros/ros.h>

#include <xpp_msgs/topic_names.h>

#include <xpp_vis/urdf_visualizer.h>
#include <xpp_vis_hyq/joints_monoped.h>
#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis_hyq/monoped_inverse_kinematics.h>

using namespace xpp;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "monoped_urdf_visualizer");

  int n_j  = KNumJointsMono;
  std::vector<std::string> joint_names(n_j);
  joint_names.at(HAA) = "haa_joint";
  joint_names.at(HFE) = "hfe_joint";
  joint_names.at(KFE) = "kfe_joint";

	auto ik = std::make_shared<MonopedInverseKinematics>();
  CartesianJointConverter inv_kin_converter(ik,
                                            xpp_msgs::robot_state_desired,
                                            xpp_msgs::joint_desired);


	std::string urdf = "monoped_rviz_urdf_robot_description";
	UrdfVisualizer node_des(joint_names, "base", urdf, "world",
	                        xpp_msgs::joint_desired, "monoped");

	// exclusively listens to this topic
//  UrdfVisualizer node_mono(ik, kMapXppJointToUrdfNames, urdf, "world",
//                      "xpp/state_mono", "monoped");


	std::cout<<"Created monoped_urdf_visualizer"<<std::endl;

	::ros::spin();

	return 1;
}

