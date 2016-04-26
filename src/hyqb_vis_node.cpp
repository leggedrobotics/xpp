/**
@file    hyqb_vis_node.cpp
@author  Diego Pardo (depardo@ethz.ch) & Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2015
@brief   
 */

#include <ros/ros.h>
#include <hyqb_vis/hyqb_vis.hpp>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "hyqb_vis");

	std::cout<<"Creating node"<<std::endl;

	hyqb::hyqb_vis node;
	node.init();

	std::cout<<"Done creating node"<<std::endl;

	ros::spin();

	return 1;
}
