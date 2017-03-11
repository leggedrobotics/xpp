/**
 @file    hyqb_vis.hpp
 @author  Diego Pardo (depardo@ethz.ch)
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 24, 2015
 @brief   Brief description here.
 */

#ifndef HYQB_VIS_HPP
#define HYQB_VIS_HPP

#include <xpp/robot_rviz_visualizer.h>

namespace xpp {

class HyqRvizVisualizer : public RobotRvizVisualizer {
public:
	HyqRvizVisualizer();
	virtual ~HyqRvizVisualizer();

	void setRobotJointsFromMessage(const sensor_msgs::JointState &msg,
	                               NameJointAngleMap& q);

	virtual void VisualizeCartesian(const xpp_msgs::RobotStateCartesian& msg);
};

} // namespace xpp

#endif /* HYQB_VIS_HPP */
