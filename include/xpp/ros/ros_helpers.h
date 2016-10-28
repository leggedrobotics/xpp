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
#include <xpp_msgs/HyqStateEETrajectory.h>
#include <xpp_msgs/HyqStateJointsTrajectory.h>
//#include <xpp_msgs/PhaseInfo.h>
//#include <xpp_msgs/Contact.h>
#include <xpp_msgs/Spline.h>
//#include <hyqb_msgs/Trajectory.h>

#include <xpp/utils/base_state.h>
#include <xpp/hyq/foothold.h>
#include <xpp/hyq/hyq_state.h>
#include <xpp/hyq/leg_data_map.h>
#include <xpp/utils/polynomial_helpers.h>
//#include <xpp/opt/phase_info.h>

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
using BaseStateMsg      = xpp_msgs::BaseState;
using HyqStateEETrajMsg = xpp_msgs::HyqStateEETrajectory;
using HyqStateEEMsg     = xpp_msgs::HyqStateEE;
using HyqStateJointsMsg      = xpp_msgs::HyqStateJoints;
using HyqStateJointsTrajMsg  = xpp_msgs::HyqStateJointsTrajectory;

//using HyqRvizStateMsg      = hyqb_msgs::RobotState;
//using HyqRvizTrajectoryMsg = hyqb_msgs::Trajectory;

using VecComPoly   = std::vector<xpp::utils::ComPolynomial>;
using SplineMsg    = xpp_msgs::Spline;
//using ContactXpp   = xpp::hyq::Contact;
//using PhaseInfoXpp = xpp::opt::PhaseInfo;
//using ContactMsg   = xpp_msgs::Contact;
//using PhaseInfoMsg = xpp_msgs::PhaseInfo;
using Polynomial   = xpp::utils::Polynomial;

static const int kHyqJointsCount = iit::HyQ::jointsCount;


static double GetDoubleFromServer(const std::string& ros_param_name) {
  double val;
  if(!::ros::param::get(ros_param_name,val))
    throw ::ros::Exception("GetDoubleFromServer: Couldn't read parameter: " + ros_param_name);
  return val;
}

static bool GetBoolFromServer(const std::string& ros_param_name) {
  bool val;
  if(!::ros::param::get(ros_param_name,val))
    throw ::ros::Exception("GetBoolFromServer: Couldn't read parameter: " + ros_param_name);
  return val;
}

static std::string GetStringFromServer(const std::string& ros_param_name) {
  std::string val;
  if(!::ros::param::get(ros_param_name,val)) {
    ROS_ERROR_STREAM("GetStringFromServer: Couldn't read parameter: " << ros_param_name);
    ROS_INFO_STREAM("\nIngore?");
    std::cin.get(); // wait for user input
//    throw ::ros::Exception("GetStringFromServer: Couldn't read parameter: " + ros_param_name);
  }
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

static HyqStateEEMsg
XppToRos(const xpp::hyq::HyqStateEE& xpp)
{
  HyqStateEEMsg ros;

  ros.base = XppToRos(xpp.base_);

  for (int leg=0; leg<4; ++leg) {
    ros.ee_in_contact[leg] = !xpp.swingleg_[leg];
    ros.endeffectors[leg]  = XppToRos(xpp.feet_[leg]);
  }

  return ros;
}

static xpp::hyq::HyqStateEE
RosToXpp(const HyqStateEEMsg& ros)
{
  xpp::hyq::HyqStateEE xpp;

  xpp.base_ = RosToXpp(ros.base);

  for (int leg=0; leg<4; ++leg) {
    xpp.swingleg_[leg] = !ros.ee_in_contact[leg];
    xpp.feet_[leg]     = RosToXpp(ros.endeffectors[leg]);
  }

  return xpp;
}

static HyqStateEETrajMsg
XppToRos(const std::vector<xpp::hyq::HyqStateEE>& xpp)
{
  HyqStateEETrajMsg msg;

  for (const auto& state : xpp)
    msg.states.push_back(XppToRos(state));

  return msg;
}

static std::vector<xpp::hyq::HyqStateEE>
RosToXpp(const HyqStateEETrajMsg& ros)
{
  std::vector<xpp::hyq::HyqStateEE> xpp;

  for (const auto& state : ros.states)
    xpp.push_back(RosToXpp(state));

  return xpp;
}

static HyqStateJointsMsg
XppToRos(const xpp::hyq::HyqStateJoints& xpp)
{
  HyqStateJointsMsg msg;
  msg.base = XppToRos(xpp.base_);

  for (int leg=0; leg<4; ++leg) {
    msg.ee_in_contact[leg] = !xpp.swingleg_[leg];
  }

  for (int j=0; j<kHyqJointsCount; ++j) {
    msg.joints.position.push_back(xpp.q(j));
    msg.joints.velocity.push_back(xpp.qd(j));
    msg.joint_acc.at(j) = xpp.qdd(j);
  }

  return msg;
}

static xpp::hyq::HyqStateJoints
RosToXpp(const HyqStateJointsMsg& msg)
{
  xpp::hyq::HyqStateJoints xpp;

  xpp.base_ = RosToXpp(msg.base);

  for (int leg=0; leg<4; ++leg) {
    xpp.swingleg_[leg] = !msg.ee_in_contact[leg];
  }

  for (int j=0; j<kHyqJointsCount; ++j) {
    xpp.q(j)   = msg.joints.position.at(j);
    xpp.qd(j)  = msg.joints.velocity.at(j);
    xpp.qdd(j) = msg.joint_acc.at(j);
  }

  return xpp;
}

static HyqStateJointsTrajMsg
XppToRos(const std::vector<xpp::hyq::HyqStateJoints>& xpp)
{
  HyqStateJointsTrajMsg msg;

  for (const auto& state : xpp) {
    msg.states.push_back(XppToRos(state));
  }

  return msg;
}

static std::vector<xpp::hyq::HyqStateJoints>
RosToXpp(const HyqStateJointsTrajMsg& msg)
{
  std::vector<xpp::hyq::HyqStateJoints> xpp;

  for (const auto& state : msg.states) {
    xpp.push_back(RosToXpp(state));
  }

  return xpp;
}

//// conversions to display hyq state in rviz using hyqb_visualizer
//static HyqRvizStateMsg
//XppToRosRviz(const xpp::hyq::HyqStateJoints& xpp)
//{
//  HyqRvizStateMsg msg;
//  msg.pose.position = XppToRos<geometry_msgs::Point>(xpp.base_.lin.p);
//  msg.twist.linear  = XppToRos<geometry_msgs::Vector3>(xpp.base_.lin.v);
//
//  msg.pose.orientation = XppToRos(xpp.base_.ang.q);
//  msg.twist.angular    = XppToRos<geometry_msgs::Vector3>(xpp.base_.ang.v);
//
//  for (int j=0; j<kHyqJointsCount; ++j)
//    msg.joints.position.push_back(xpp.q(j));
//
//  return msg;
//}
//
//static HyqRvizTrajectoryMsg
//XppToRosRviz(const std::vector<xpp::hyq::HyqStateJoints>& xpp)
//{
//  HyqRvizTrajectoryMsg msg;
//
//  msg.dt.data = 0.004; // task servo rate
//  for (const auto& state : xpp) {
//    msg.states.push_back(XppToRosRviz(state));
//  }
//
//  return msg;
//}


static std::vector<SplineMsg>
XppToRos(const VecComPoly& opt_splines)
{
  int n_splines = opt_splines.size();
  std::vector<SplineMsg> msgs(n_splines);

  for (uint i=0; i<opt_splines.size(); ++i)
  {

    for (auto coeff : Polynomial::AllSplineCoeff) {
      msgs.at(i).coeff_x[coeff] = opt_splines.at(i).GetCoefficient(xpp::utils::X,coeff);
      msgs.at(i).coeff_y[coeff] = opt_splines.at(i).GetCoefficient(xpp::utils::Y,coeff);
    }

    msgs.at(i).duration = opt_splines.at(i).GetDuration();
    msgs.at(i).id       = opt_splines.at(i).GetId();
  }

  return msgs;
}

static VecComPoly
RosToXpp(const std::vector<SplineMsg>& msgs)
{
  uint n_splines = msgs.size();
  VecComPoly xpp(n_splines);

  for (uint i=0; i<n_splines; ++i)
  {
    for (auto coeff : Polynomial::AllSplineCoeff) {
      xpp.at(i).SetCoefficients(xpp::utils::X, coeff, msgs.at(i).coeff_x[coeff]);
      xpp.at(i).SetCoefficients(xpp::utils::Y, coeff, msgs.at(i).coeff_y[coeff]);
    }
    xpp.at(i).SetDuration(msgs.at(i).duration);
    xpp.at(i).SetId(msgs.at(i).id);
  }
  return xpp;
}

//static ContactMsg
//XppToRos(const ContactXpp& xpp)
//{
//  ContactMsg msg;
//  msg.id = xpp.id;
//  msg.ee = static_cast<int>(xpp.ee);
//
//  return msg;
//}
//
//static ContactXpp
//RosToXpp(const ContactMsg& msg)
//{
//  ContactXpp xpp;
//  xpp.id = msg.id;
//  xpp.ee = static_cast<xpp::opt::EndeffectorID>(msg.ee);
//
//  return xpp;
//}
//
//static PhaseInfoMsg
//XppToRos(const PhaseInfoXpp& xpp)
//{
//  PhaseInfoMsg msg;
//  msg.n_completed_steps = xpp.n_completed_steps_;
//  for (auto c : xpp.free_contacts_)  msg.free_contacts.push_back(XppToRos(c));
//  for (auto f : xpp.fixed_contacts_) msg.fixed_contacts.push_back(xpp::ros::RosHelpers::XppToRos(f));
//  msg.id                = xpp.id_;
//  msg.duration          = xpp.duration_;
//
//  return msg;
//}
//
//static PhaseInfoXpp
//RosToXpp(const PhaseInfoMsg& msg)
//{
//  PhaseInfoXpp xpp;
//  xpp.n_completed_steps_ = msg.n_completed_steps;
//  for (auto c : msg.free_contacts)  xpp.free_contacts_.push_back(RosToXpp(c));
//  for (auto f : msg.fixed_contacts) xpp.fixed_contacts_.push_back(xpp::ros::RosHelpers::RosToXpp(f));
//  xpp.id_                = msg.id;
//  xpp.duration_          = msg.duration;
//
//  return xpp;
//}
//
//static std::vector<PhaseInfoMsg>
//XppToRos(const std::vector<PhaseInfoXpp>& xpp)
//{
//  std::vector<PhaseInfoMsg> msg;
//
//  for (const auto& phase : xpp)
//    msg.push_back(XppToRos(phase));
//
//  return msg;
//}
//
//static std::vector<PhaseInfoXpp>
//RosToXpp(const std::vector<PhaseInfoMsg>& msg)
//{
//  std::vector<PhaseInfoXpp> xpp;
//
//  for (auto phase : msg)
//    xpp.push_back(RosToXpp(phase));
//
//  return xpp;
//}

}; // RosHelpers

} // namespace ros
} // namespace xpp

#endif /* INCLUDE_XPP_MSGS_ROS_HELPERS_H_ */
