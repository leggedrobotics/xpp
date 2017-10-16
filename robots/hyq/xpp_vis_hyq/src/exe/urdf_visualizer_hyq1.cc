
#include <ros/ros.h>

#include <xpp_vis_hyq/inverse_kinematics_hyq1.h>
#include <xpp_msgs/topic_names.h>

#include <xpp_vis/urdf_visualizer.h>
#include <xpp_vis/cartesian_joint_converter.h>

using namespace xpp;

int main(int argc, char *argv[])
{
	::ros::init(argc, argv, "monoped_urdf_visualizer");

	auto ik = std::make_shared<InverseKinematicsHyq1>();
  CartesianJointConverter inv_kin_converter(ik,
                                            xpp_msgs::robot_state_desired,
                                            xpp_msgs::joint_desired);

  std::vector<UrdfVisualizer::URDFName> joint_names(HyqlegJointCount);
  joint_names.at(HAA) = "haa_joint";
  joint_names.at(HFE) = "hfe_joint";
  joint_names.at(KFE) = "kfe_joint";

	std::string urdf = "monoped_rviz_urdf_robot_description";
	UrdfVisualizer node_des(urdf, joint_names, "base", "world",
	                        xpp_msgs::joint_desired, "monoped");

	::ros::spin();

	return 1;
}

