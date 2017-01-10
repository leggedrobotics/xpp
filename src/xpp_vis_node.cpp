/**
@file    hyqb_vis_node.cpp
@author  Diego Pardo (depardo@ethz.ch) & Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2015
@brief   
 */

#include <ros/ros.h>

#include "../include/xpp/vis/hyq_rviz_visualizer.hpp"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "hyqb_vis");

	xpp::vis::HyqRvizVisualizer node;
	node.init();

	std::cout<<"Created hyqb_visualizer node"<<std::endl;

	ros::spin();

	return 1;
}
