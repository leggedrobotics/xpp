/*
 * ros_helpers.h
 *
 *  Created on: Apr 8, 2016
 *      Author: winklera
 */

#ifndef INCLUDE_XPP_MSGS_ROS_HELPERS_H_
#define INCLUDE_XPP_MSGS_ROS_HELPERS_H_

#include <ros/ros.h>

#include <xpp_msgs/StateLin3d.h>
#include <xpp_msgs/Foothold.h>
#include <xpp_msgs/RobotStateTrajectoryCartesian.h>
#include <xpp_msgs/HyqStateTrajectory.h>
#include <hyqb_msgs/Trajectory.h>

#include <xpp/utils/base_state.h>
#include <xpp/hyq/foothold.h>
#include <xpp/hyq/hyq_state.h>
#include <xpp/hyq/leg_data_map.h>

namespace xpp {
namespace ros {

/**
 * Ros specific functions that only depend on utils folder, ros messages,
 * ros services.
 */
struct RosHelpers {

typedef Eigen::Vector3d Vector3d;
typedef xpp::utils::BaseLin3d State;
typedef xpp::hyq::Foothold Foothold;
typedef xpp::hyq::LegID LegID;

// Aliases for all ros messages
using FootholdMsg       = xpp_msgs::Foothold;
using StateLin3dMsg     = xpp_msgs::StateLin3d;
using RobotStateTrajMsg = xpp_msgs::RobotStateTrajectoryCartesian;
using RobotStateMsg     = xpp_msgs::RobotStateCartesianStamped;
using BaseStateMsg      = xpp_msgs::BaseState;
using RobotStateJoint   = hyqb_msgs::RobotState;
using HyqStateMsg       = xpp_msgs::HyqState;
using HyqStateTrajMsg   = xpp_msgs::HyqStateTrajectory;
using RobotStateJointTrajMsg = hyqb_msgs::Trajectory;

static double GetDoubleFromServer(const std::string& ros_param_name) {
  double val;
  if(!::ros::param::get(ros_param_name,val))
    throw ::ros::Exception("GetDoubleFromServer: Couldn't read parameter: " + ros_param_name);
  return val;
}

static double GetBoolFromServer(const std::string& ros_param_name) {
  bool val;
  if(!::ros::param::get(ros_param_name,val))
    throw ::ros::Exception("GetBoolFromServer: Couldn't read parameter: " + ros_param_name);
  return val;
}

static State
RosToXpp(const StateLin3dMsg& ros)
{
  State point;
  point.p.x() = ros.pos.x;
  point.p.y() = ros.pos.y;
  point.p.z() = ros.pos.z;

  point.v.x() = ros.vel.x;
  point.v.y() = ros.vel.y;
  point.v.z() = ros.vel.z;

  point.a.x() = ros.acc.x;
  point.a.y() = ros.acc.y;
  point.a.z() = ros.acc.z;

  return point;
}

static StateLin3dMsg
XppToRos(const State& xpp)
{
  StateLin3dMsg ros;
  ros.pos.x = xpp.p.x();
  ros.pos.y = xpp.p.y();
  ros.pos.z = xpp.p.z();

  ros.vel.x = xpp.v.x();
  ros.vel.y = xpp.v.y();
  ros.vel.z = xpp.v.z();

  ros.acc.x = xpp.a.x();
  ros.acc.y = xpp.a.y();
  ros.acc.z = xpp.a.z();

  return ros;
}

template<typename T>
static Vector3d
RosToXpp(const T& ros)
{
  Vector3d vec;
  vec << ros.x, ros.y, ros.z;
  return vec;
}

template<typename T>
static T
XppToRos(const Vector3d& xpp)
{
  T ros;
  ros.x = xpp.x();
  ros.y = xpp.y();
  ros.z = xpp.z();

  return ros;
}

static Foothold
RosToXpp(const FootholdMsg& ros)
{
  Foothold f;

  assert(0 <= ros.leg && ros.leg < xpp::hyq::_LEGS_COUNT); //integer cannot be mapped to a LegID
  f.leg = static_cast<LegID>(ros.leg);
  f.p   = RosToXpp(ros.p);
  return f;
}

static FootholdMsg
XppToRos(const xpp::hyq::Foothold& xpp)
{
  FootholdMsg ros;
  ros.p.x = xpp.p.x();
  ros.p.y = xpp.p.y();
  ros.p.z = xpp.p.z();
  ros.leg = static_cast<int>(xpp.leg);

  return ros;
}

static std::vector<FootholdMsg>
XppToRos(const std::vector<xpp::hyq::Foothold>& xpp)
{
  int n_footholds = xpp.size();
  std::vector<FootholdMsg> ros_vec(n_footholds);

  for (int i=0; i<n_footholds; ++i) {
    ros_vec.at(i) = XppToRos(xpp.at(i));
  }

  return ros_vec;
}

static std::vector<xpp::hyq::Foothold>
RosToXpp(const std::vector<FootholdMsg>& ros)
{
  std::vector<xpp::hyq::Foothold> xpp_vec(ros.size());

  for (uint i=0; i<ros.size(); ++i) {
    xpp_vec.at(i) = RosToXpp(ros.at(i));
  }

  return xpp_vec;
}

static Eigen::Quaterniond
RosToXpp(const geometry_msgs::Quaternion ros)
{
  Eigen::Quaterniond xpp;
  xpp.w() = ros.w;
  xpp.x() = ros.x;
  xpp.y() = ros.y;
  xpp.z() = ros.z;

  return xpp;
}

static geometry_msgs::Quaternion
XppToRos(const Eigen::Quaterniond xpp)
{
  geometry_msgs::Quaternion ros;
  ros.w = xpp.w();
  ros.x = xpp.x();
  ros.y = xpp.y();
  ros.z = xpp.z();

  return ros;
}

static BaseStateMsg
XppToRos(const xpp::utils::BaseState& xpp)
{
  BaseStateMsg msg;

  msg.pose.position = XppToRos<geometry_msgs::Point>(xpp.lin.p);
  msg.twist.linear  = XppToRos<geometry_msgs::Vector3>(xpp.lin.v);
  msg.accel.linear  = XppToRos<geometry_msgs::Vector3>(xpp.lin.a);

  msg.pose.orientation = XppToRos(xpp.ang.q);
  msg.twist.angular    = XppToRos<geometry_msgs::Vector3>(xpp.ang.v);
  msg.accel.angular    = XppToRos<geometry_msgs::Vector3>(xpp.ang.a);

  return msg;
}

static xpp::utils::BaseState
RosToXpp(const BaseStateMsg& ros)
{
  xpp::utils::BaseState xpp;

  xpp.lin.p = RosToXpp(ros.pose.position);
  xpp.lin.v = RosToXpp(ros.twist.linear);
  xpp.lin.a = RosToXpp(ros.accel.linear);

  xpp.ang.q = RosToXpp(ros.pose.orientation);
  xpp.ang.v = RosToXpp(ros.twist.angular);
  xpp.ang.a = RosToXpp(ros.accel.angular);

  return xpp;
}

static RobotStateMsg
XppToRos(const xpp::hyq::HyqStateStamped& xpp)
{
  RobotStateMsg ros;

  ros.state.base = XppToRos(xpp.base_);
  ros.time       = xpp.t_;

  for (int leg=0; leg<4; ++leg) {
    ros.state.ee_in_contact[leg] = !xpp.swingleg_[leg];
    ros.state.endeffectors[leg]  = XppToRos(xpp.feet_[leg]);
  }

  return ros;
}

static xpp::hyq::HyqStateStamped
RosToXpp(const RobotStateMsg& ros)
{
  xpp::hyq::HyqStateStamped xpp;

  xpp.base_ = RosToXpp(ros.state.base);
  xpp.t_    = ros.time;

  for (int leg=0; leg<4; ++leg) {
    xpp.swingleg_[leg] = !ros.state.ee_in_contact[leg];
    xpp.feet_[leg]     = RosToXpp(ros.state.endeffectors[leg]);
  }

  return xpp;
}

static RobotStateTrajMsg
XppToRos(const std::vector<xpp::hyq::HyqStateStamped>& xpp)
{
  RobotStateTrajMsg msg;

  for (const auto& state : xpp)
    msg.states.push_back(XppToRos(state));

  return msg;
}

static std::vector<xpp::hyq::HyqStateStamped>
RosToXpp(const RobotStateTrajMsg& ros)
{
  std::vector<xpp::hyq::HyqStateStamped> xpp;

  for (const auto& state : ros.states)
    xpp.push_back(RosToXpp(state));

  return xpp;
}

static RobotStateJoint
XppToRos(const xpp::hyq::HyQStateJoints& xpp)
{
  RobotStateJoint msg;
  msg.pose.position = XppToRos<geometry_msgs::Point>(xpp.base_.lin.p);
  msg.twist.linear  = XppToRos<geometry_msgs::Vector3>(xpp.base_.lin.v);

  msg.pose.orientation = XppToRos(xpp.base_.ang.q);
  msg.twist.angular    = XppToRos<geometry_msgs::Vector3>(xpp.base_.ang.v);

  for (int j=0; j<xpp::hyq::jointsCount; ++j)
    msg.joints.position.push_back(xpp.q(j));

  return msg;
}

static RobotStateJointTrajMsg
XppToRos(const std::vector<xpp::hyq::HyQStateJoints>& xpp)
{
  RobotStateJointTrajMsg msg;

  msg.dt.data = 0.004;
  for (const auto& state : xpp) {
    msg.states.push_back(XppToRos(state));
  }

  return msg;
}

// inv_kin rename this to standard and change signature above to special
static HyqStateMsg
XppToRosHyq(const xpp::hyq::HyQStateJoints& xpp)
{
  HyqStateMsg msg;
  msg.base = XppToRos(xpp.base_);

  for (int leg=0; leg<4; ++leg) {
    msg.ee_in_contact[leg] = !xpp.swingleg_[leg];
  }

  for (int j=0; j<xpp::hyq::jointsCount; ++j) {
    msg.joints.position.push_back(xpp.q(j));
    msg.joints.velocity.push_back(xpp.qd(j));
    msg.joint_acc.at(j) = xpp.qdd(j);
  }

  return msg;
}

static xpp::hyq::HyQStateJoints
RosToXpp(const HyqStateMsg& msg)
{
  xpp::hyq::HyQStateJoints xpp;

  xpp.base_ = RosToXpp(msg.base);

  for (int leg=0; leg<4; ++leg) {
    xpp.swingleg_[leg] = !msg.ee_in_contact[leg];
  }

  for (int j=0; j<xpp::hyq::jointsCount; ++j) {
    xpp.q(j)   = msg.joints.position.at(j);
    xpp.qd(j)  = msg.joints.velocity.at(j);
    xpp.qdd(j) = msg.joint_acc.at(j);
  }

  return xpp;
}

static HyqStateTrajMsg
XppToRosHyq(const std::vector<xpp::hyq::HyQStateJoints>& xpp)
{
  HyqStateTrajMsg msg;

  msg.dt.data = 0.004;
  for (const auto& state : xpp) {
    msg.states.push_back(XppToRosHyq(state));
  }

  return msg;
}

static std::vector<xpp::hyq::HyQStateJoints>
RosToXpp(const HyqStateTrajMsg& msg)
{
  std::vector<xpp::hyq::HyQStateJoints> xpp;

  for (const auto& state : msg.states) {
    xpp.push_back(RosToXpp(state));
  }

  return xpp;
}

}; // RosHelpers

} // namespace ros
} // namespace xpp

#endif /* INCLUDE_XPP_MSGS_ROS_HELPERS_H_ */
