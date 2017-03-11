/**
@file    xpp_to_ros.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Mar 10, 2017
@brief   Brief description
 */

#ifndef XPP_XPP_VIS_INCLUDE_XPP_VIS_ROS_HELPERS_JOINTS_H_
#define XPP_XPP_VIS_INCLUDE_XPP_VIS_ROS_HELPERS_JOINTS_H_

#include <xpp/exe/robot_state_joints.h>
#include <xpp/ros/ros_helpers.h>

#include <xpp_msgs/RobotStateJoints.h>
#include <xpp_msgs/RobotStateJointsTrajectory.h>

namespace xpp {

struct RosHelpersJoints {

using RobotStateJointsMsg     = xpp_msgs::RobotStateJoints;

static RobotStateJointsMsg
XppToRos(const RobotStateJoints& xpp)
{
  RobotStateJointsMsg ros;

  ros.common = ros::RosHelpers::XppToRos(xpp.GetCommon());

  for (int i=0; i<xpp.GetJointCount(); ++i) {
    JointID j = static_cast<JointID>(i);
    ros.joints.position.push_back(xpp.GetJointPos().At(j));
    ros.joints.velocity.push_back(xpp.GetJointVel().At(j));
    ros.joint_acc.push_back(xpp.GetJointAcc().At(j));
  }

  ros.n_joints_per_ee = xpp.GetJointsPerEE();

  return ros;
}

//static RobotStateJoints
//RosToXpp(const RobotStateJointsMsg& msg)
//{
//  int n_ee            = msg.common.ee_in_contact.size();
//  int n_joints_per_ee = msg.n_joints_per_ee;
//  int n_joints        = msg.joint_acc.size();
//
//  RobotStateJoints xpp(n_ee, n_joints_per_ee);
//  xpp.SetCommon(RosHelpers::RosToXpp(msg.common));
//
//  auto joint_pos = xpp.GetJointPos();
//  auto joint_vel = xpp.GetJointVel();
//  auto joint_acc = xpp.GetJointAcc();
//  for (int i=0; i<n_joints; ++i) {
//    xpp::utils::JointID j = static_cast<xpp::utils::JointID>(i);
//    joint_pos.At(j) = msg.joints.position.at(i);
//    joint_vel.At(j) = msg.joints.velocity.at(i);
//    joint_acc.At(j) = msg.joint_acc.at(i);
//  }
//
//  xpp.SetJointPos(joint_pos);
//  xpp.SetJointVel(joint_vel);
//  xpp.SetJointAcc(joint_acc);
//
//  return xpp;
//}

//using RobotStateJointsTrajMsg = xpp_msgs::RobotStateJointsTrajectory;
//
//static RobotStateJointsTrajMsg
//XppToRosJoints(const std::vector<RobotStateJoints>& xpp)
//{
//  return RosHelpers::XppToRos<std::vector<RobotStateJoints>,RobotStateJointsTrajMsg>(xpp);
//}
//
//static std::vector<RobotStateJoints>
//RosToXppJoints(const RobotStateJointsTrajMsg& ros)
//{
//  return RosHelpers::RosToXpp<RobotStateJointsTrajMsg,std::vector<RobotStateJoints> >(ros);
//}

}; // RosHelpersJoints

} // namespace xpp

#endif /* XPP_XPP_VIS_INCLUDE_XPP_VIS_ROS_HELPERS_JOINTS_H_ */
