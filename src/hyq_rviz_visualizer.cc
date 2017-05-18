/**
@file    hyq_rviz_visualizer.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Mar 11, 2017
 */

#include <xpp/hyq_rviz_visualizer.h>

#include <xpp/hyq/joints_hyq.h>
#include <xpp/hyq/hyq_inverse_kinematics.h>
#include <xpp/ros_helpers_joints.h>

namespace xpp {

void
HyqRvizVisualizer::SetJointsFromRos(const sensor_msgs::JointState &msg,
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

void
HyqRvizVisualizer::StateCallback(const StateMsg& msg)
{
  auto hyq_ik = std::make_shared<hyq::HyqInverseKinematics>();
  auto cart   = xpp::ros::RosHelpers::RosToXpp(msg);
  auto joints = RobotStateJoints::FromCartesian(cart, hyq_ik);

  auto joints_msg = xpp::RosHelpersJoints::XppToRos(joints);
  VisualizeJoints(::ros::Time::now(), joints_msg.common.base.pose,
                                      joints_msg.joints);
}


}



