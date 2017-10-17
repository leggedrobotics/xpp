
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_vis_hyq/inverse_kinematics_hyq4.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "hyq_urdf_visualizer");

	const std::string joint_desired_hyq = "xpp/joint_hyq_des";

	auto hyq_ik = std::make_shared<InverseKinematicsHyq4>();
	CartesianJointConverter inv_kin_converter(hyq_ik,
	                                          xpp_msgs::robot_state_desired,
	                                          joint_desired_hyq);

	// urdf joint names
	int n_ee = hyq_ik->GetEECount();
	int n_j  = HyqlegJointCount;
	std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
	joint_names.at(n_j*LF + HAA) = "lf_haa_joint";
	joint_names.at(n_j*LF + HFE) = "lf_hfe_joint";
	joint_names.at(n_j*LF + KFE) = "lf_kfe_joint";
	joint_names.at(n_j*RF + HAA) = "rf_haa_joint";
	joint_names.at(n_j*RF + HFE) = "rf_hfe_joint";
	joint_names.at(n_j*RF + KFE) = "rf_kfe_joint";
	joint_names.at(n_j*LH + HAA) = "lh_haa_joint";
	joint_names.at(n_j*LH + HFE) = "lh_hfe_joint";
	joint_names.at(n_j*LH + KFE) = "lh_kfe_joint";
	joint_names.at(n_j*RH + HAA) = "rh_haa_joint";
	joint_names.at(n_j*RH + HFE) = "rh_hfe_joint";
	joint_names.at(n_j*RH + KFE) = "rh_kfe_joint";

	std::string urdf = "hyq_rviz_urdf_robot_description";
	UrdfVisualizer hyq_desired(urdf, joint_names, "base", "world",
	                           joint_desired_hyq, "hyq_des");

	::ros::spin();

	return 1;
}

