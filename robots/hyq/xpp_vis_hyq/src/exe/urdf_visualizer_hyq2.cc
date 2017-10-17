
#include <ros/ros.h>

#include <xpp_vis_hyq/inverse_kinematics_hyq2.h>
#include <xpp_msgs/topic_names.h>

#include <xpp_vis/urdf_visualizer.h>
#include <xpp_vis/cartesian_joint_converter.h>

#include <xpp_states/endeffector_mappings.h>

using namespace xpp;
using namespace biped;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "biped_urdf_visualizer");

	const std::string joint_desired_biped = "xpp/joint_biped_des";

	auto ik = std::make_shared<InverseKinematicsHyq2>();
  CartesianJointConverter inv_kin_converter(ik,
                                            xpp_msgs::robot_state_desired,
                                            joint_desired_biped);

  int n_ee = ik->GetEECount();
  int n_j  = HyqlegJointCount;
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
  joint_names.at(n_j*L + HAA) = "L_haa_joint";
  joint_names.at(n_j*L + HFE) = "L_hfe_joint";
  joint_names.at(n_j*L + KFE) = "L_kfe_joint";
  joint_names.at(n_j*R + HAA) = "R_haa_joint";
  joint_names.at(n_j*R + HFE) = "R_hfe_joint";
  joint_names.at(n_j*R + KFE) = "R_kfe_joint";

	std::string urdf = "biped_rviz_urdf_robot_description";
	UrdfVisualizer node(urdf, joint_names, "base", "world",
	                    joint_desired_biped, "biped");

	::ros::spin();

	return 1;
}

