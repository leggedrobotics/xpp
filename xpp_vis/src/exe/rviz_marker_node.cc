/**
@file    opt_state_visualizer_node.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.05.2017
@brief   Visualizes a single optimization state in rviz.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <xpp_msgs/UserCommand.h>      // listen to goal state
#include <xpp/rviz_marker_builder.h>
#include <xpp_ros_conversions/ros_conversions.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>


using namespace ros;


static Publisher rviz_marker_pub;
static ros::Publisher rviz_pose_pub;
static xpp::RvizMarkerBuilder marker_builder;

static void StateCallback (const xpp_msgs::RobotStateCartesian& state_msg)
{
  auto rviz_marker_msg = marker_builder.BuildStateMarkers(state_msg);
  rviz_marker_pub.publish(rviz_marker_msg);
}

static void TerrainInfoCallback (const xpp_msgs::TerrainInfo& terrain_msg)
{
  marker_builder.terrain_msg_ = terrain_msg;
}

static void ParamsCallback (const xpp_msgs::OptParameters& params_msg)
{
  ROS_DEBUG_STREAM("received current optimization parameters");
  marker_builder.SetOptimizationParameters(params_msg);
}

void CallbackUserCommand(const xpp_msgs::UserCommand& msg_in)
{
  // add some terrain
  auto msg = marker_builder.BuildTerrain(msg_in.terrain_id);
  rviz_marker_pub.publish(msg);

  // publish goal pose
  auto goal_msg = marker_builder.BuildGoalPose(msg_in.goal_lin.pos,
                                               msg_in.goal_ang);
  rviz_pose_pub.publish(goal_msg);
}


int main(int argc, char *argv[])
{
	init(argc, argv, "rviz_marker_visualizer");

	NodeHandle n;

  Subscriber parameters_sub;
  parameters_sub = n.subscribe(xpp_msgs::opt_parameters, 1, ParamsCallback);

	Subscriber state_sub_curr, state_sub_des, terrain_info_sub;
	state_sub_curr    = n.subscribe(xpp_msgs::robot_state_current, 1, StateCallback);
	state_sub_des     = n.subscribe(xpp_msgs::robot_state_desired, 1, StateCallback);
	terrain_info_sub  = n.subscribe(xpp_msgs::terrain_info, 1,  TerrainInfoCallback);

	Subscriber goal_sub;
	goal_sub = n.subscribe(xpp_msgs::user_command, 1, CallbackUserCommand);

	// publishers
	rviz_marker_pub = n.advertise<visualization_msgs::MarkerArray>("xpp/rviz_markers", 1);
	rviz_pose_pub   = n.advertise<geometry_msgs::PoseStamped>("xpp/goal", 1);

	std::cout<<"Created rviz marker visualizer"<<std::endl;

	spin();

	return 1;
}


