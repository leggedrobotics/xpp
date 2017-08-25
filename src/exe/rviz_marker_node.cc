/**
@file    opt_state_visualizer_node.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.05.2017
@brief   Visualizes a single optimization state in rviz.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <kindr/Core>

#include <xpp_msgs/UserCommand.h>      // listen to goal state
#include <xpp/rviz_marker_builder.h>
#include <xpp/ros/ros_conversions.h>
#include <xpp/ros/topic_names.h>

using PoseMsg        = geometry_msgs::PoseStamped;
using StateMsg       = xpp_msgs::RobotStateCartesian;
using ParamsMsg      = xpp_msgs::OptParameters;
using MarkerArrayMsg = visualization_msgs::MarkerArray;

using namespace ros;

static Publisher rviz_marker_pub;
static ros::Publisher rviz_pose_pub;
static xpp::RvizMarkerBuilder marker_builder;

static void StateCallback (const StateMsg& state_msg)
{
  auto state = xpp::ros::RosConversions::RosToXpp(state_msg);
  MarkerArrayMsg rviz_marker_msg = marker_builder.BuildStateMarkers(state);
  rviz_marker_pub.publish(rviz_marker_msg);
}

static void ParamsCallback (const ParamsMsg& params_msg)
{
  ROS_INFO_STREAM("received current optimization parameters");
  marker_builder.SetOptimizationParameters(params_msg);
}

void CallbackUserCommand(const xpp_msgs::UserCommand& msg_in)
{
  // publish goal pose
  PoseMsg msg_out;
  msg_out.header.frame_id = "world";
  msg_out.pose.position   = msg_in.goal_lin.pos;

  auto goal_ang = xpp::ros::RosConversions::RosToXpp(msg_in.goal_ang);
  kindr::EulerAnglesZyxD euler(goal_ang.p_.reverse());
  kindr::RotationQuaternionD quat(euler);
  msg_out.pose.orientation = xpp::ros::RosConversions::XppToRos(quat.toImplementation());

  rviz_pose_pub.publish(msg_out);

  // add some terrain
  MarkerArrayMsg msg = marker_builder.BuildTerrain(msg_in.terrain_id);
  rviz_marker_pub.publish(msg);
}


int main(int argc, char *argv[])
{
	init(argc, argv, "rviz_marker_visualizer");

	NodeHandle n;

  Subscriber parameters_sub;
  parameters_sub = n.subscribe(xpp_msgs::opt_parameters, 1, ParamsCallback);

	Subscriber state_sub;
	state_sub = n.subscribe(xpp_msgs::robot_state, 1, StateCallback);

	Subscriber goal_sub;
	goal_sub = n.subscribe(xpp_msgs::user_command, 1, CallbackUserCommand);

	// publishers
	rviz_marker_pub = n.advertise<MarkerArrayMsg>("xpp/rviz_markers", 1);
	rviz_pose_pub = n.advertise<PoseMsg>("xpp/goal", 1);


	std::cout<<"Created rviz marker visualizer"<<std::endl;

	spin();

	return 1;
}


