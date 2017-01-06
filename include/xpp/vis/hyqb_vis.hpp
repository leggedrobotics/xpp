/**
 @file    hyqb_vis.hpp
 @author  Diego Pardo (depardo@ethz.ch)
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 24, 2015
 @brief   Brief description here.
 */

#ifndef HYQB_VIS_HPP
#define HYQB_VIS_HPP

#include "robotVisBase.hpp"
#include <xpp/hyq/q_hyq.h>

namespace xpp {
namespace vis {

class hyqb_vis : public robotVisBase<hyq::kNumJoints> {
public:
	hyqb_vis() :robotVisBase("hyq", xpp::hyq::kJointNamesVis) {};
	virtual ~hyqb_vis() {};

	void setRobotJointsFromMessage(const sensor_msgs::JointState &msg,
	                               NameJointAngleMap& q)
	{
	  for(size_t i = 0 ; i < hyq::kNumJoints; i++) {
	    // message always ordered by xpp
	    auto index_xpp = xpp::hyq::kMapHyqToXpp.at(static_cast<xpp::hyq::HyqJointID>(i));

	    q[robot_joint_names.at(i)] = msg.position[index_xpp];
	  }
	}

};

} // namespace vis
} // namespace xpp

#endif /* HYQB_VIS_HPP */
