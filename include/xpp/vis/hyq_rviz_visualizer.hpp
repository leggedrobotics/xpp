/**
 @file    hyqb_vis.hpp
 @author  Diego Pardo (depardo@ethz.ch)
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 24, 2015
 @brief   Brief description here.
 */

#ifndef HYQB_VIS_HPP
#define HYQB_VIS_HPP

#include <xpp/hyq/joints_hyq.h>
#include "robot_rviz_visualizer.h"

namespace xpp {
namespace vis {

class HyqRvizVisualizer : public RobotRvizVisualizer {
public:
	HyqRvizVisualizer() :RobotRvizVisualizer("hyq") {
	  playbackSpeed_ = 2.0;//0.25;
	  get_curr_from_vis_ = true;
	};

	virtual ~HyqRvizVisualizer() {};

	void setRobotJointsFromMessage(const sensor_msgs::JointState &msg,
	                               NameJointAngleMap& q)
	{
	  using namespace hyq;
	  static const std::map<HyqJointID, std::string > kMapHyqJointToString {
	    { LF_HAA,  "lf_haa_joint" },
	    { LF_HFE,  "lf_hfe_joint" },
	    { LF_KFE,  "lf_kfe_joint" },
	    { RF_HAA,  "rf_haa_joint" },
	    { RF_HFE,  "rf_hfe_joint" },
	    { RF_KFE,  "rf_kfe_joint" },
	    { LH_HAA,  "lh_haa_joint" },
	    { LH_HFE,  "lh_hfe_joint" },
	    { LH_KFE,  "lh_kfe_joint" },
	    { RH_HAA,  "rh_haa_joint" },
	    { RH_HFE,  "rh_hfe_joint" },
	    { RH_KFE,  "rh_kfe_joint" }
	  };

	  for(size_t i = 0 ; i < hyq::kNumJoints; i++) {
	    // message always ordered by xpp
	    HyqJointID idx_hyq = static_cast<HyqJointID>(i);
	    auto index_xpp = kMapHyqToXpp.at(idx_hyq);

	    q[kMapHyqJointToString.at(idx_hyq)] = msg.position[index_xpp];
	  }
	}

};

} // namespace vis
} // namespace xpp

#endif /* HYQB_VIS_HPP */
