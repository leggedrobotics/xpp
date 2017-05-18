/**
@file    opt_state_visualizer_node.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.05.2017
@brief   Visualizes a single optimization state in rviz.
 */

#include <ros/ros.h>

#include <xpp/rviz_marker_builder.h>
#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>

using StateMsg       = xpp_msgs::RobotStateCartesian;
using MarkerArrayMsg = visualization_msgs::MarkerArray;

using namespace ros;

static Publisher rviz_pub;

static void StateCallback (const StateMsg& state_msg)
{
  xpp::RobotStateCartesian state = xpp::ros::RosHelpers::RosToXpp(state_msg);

  xpp::RvizMarkerBuilder msg_builder;
  MarkerArrayMsg rviz_marker_msg = msg_builder.VisualizeState(state);

  rviz_pub.publish(rviz_marker_msg);
}


int main(int argc, char *argv[])
{
	init(argc, argv, "optimization_state_visualizer");

	NodeHandle n;
	rviz_pub = n.advertise<MarkerArrayMsg>(xpp_msgs::rviz_optimized, 1);

	Subscriber state_sub;
	state_sub = n.subscribe(xpp_msgs::curr_robot_state, 1, StateCallback);

	spin();

	return 1;
}


