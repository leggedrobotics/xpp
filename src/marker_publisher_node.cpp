/**
@file    hyqb_vis_node.cpp
@author  Diego Pardo (depardo@ethz.ch) & Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2015
@brief   
 */

#include <ros/ros.h>

#include <xpp/ros/ros_visualizer.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "optimization_visualizer");

	xpp::ros::RosVisualizer node;

	ROS_INFO_STREAM("Created optimization visualizer node");

	ros::spin();

	return 1;
}
