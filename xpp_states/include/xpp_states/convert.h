/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef XPP_ROS_CONVERSIONS_H_
#define XPP_ROS_CONVERSIONS_H_

#include <vector>

#include <xpp_msgs/StateLin3d.h>
#include <xpp_msgs/State6d.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>

#include <xpp_states/state.h>
#include <xpp_states/robot_state_cartesian.h>

namespace xpp {

/**
 * @brief Converts between xpp-states types and xpp-messages.
 *
 * ToXpp() : Convert from ROS message (xpp_msgs) to state (xpp_states).
 * ToRos() : Convert from state (xpp_states) to ROS message (xpp_msgs).
 */
struct Convert {

static StateLin3d
ToXpp(const xpp_msgs::StateLin3d& ros)
{
  StateLin3d point;
  point.p_.x() = ros.pos.x;
  point.p_.y() = ros.pos.y;
  point.p_.z() = ros.pos.z;

  point.v_.x() = ros.vel.x;
  point.v_.y() = ros.vel.y;
  point.v_.z() = ros.vel.z;

  point.a_.x() = ros.acc.x;
  point.a_.y() = ros.acc.y;
  point.a_.z() = ros.acc.z;

  return point;
}

static xpp_msgs::StateLin3d
ToRos(const StateLin3d& xpp)
{
  xpp_msgs::StateLin3d ros;
  ros.pos.x = xpp.p_.x();
  ros.pos.y = xpp.p_.y();
  ros.pos.z = xpp.p_.z();

  ros.vel.x = xpp.v_.x();
  ros.vel.y = xpp.v_.y();
  ros.vel.z = xpp.v_.z();

  ros.acc.x = xpp.a_.x();
  ros.acc.y = xpp.a_.y();
  ros.acc.z = xpp.a_.z();

  return ros;
}

template<typename T>
static Vector3d
ToXpp(const T& ros)
{
  Vector3d vec;
  vec << ros.x, ros.y, ros.z;
  return vec;
}

template<typename T>
static T
ToRos(const Vector3d& xpp)
{
  T ros;
  ros.x = xpp.x();
  ros.y = xpp.y();
  ros.z = xpp.z();

  return ros;
}

template<typename T>
static Endeffectors<Vector3d>
ToXpp(const std::vector<T>& ros)
{
  Endeffectors<Vector3d> xpp(ros.size());

  for (auto ee : xpp.GetEEsOrdered())
    xpp.at(ee) = ToXpp(ros.at(ee));

  return xpp;
}

static Eigen::Quaterniond
ToXpp(const geometry_msgs::Quaternion ros)
{
  Eigen::Quaterniond xpp;
  xpp.w() = ros.w;
  xpp.x() = ros.x;
  xpp.y() = ros.y;
  xpp.z() = ros.z;

  return xpp;
}

static geometry_msgs::Quaternion
ToRos(const Eigen::Quaterniond xpp)
{
  geometry_msgs::Quaternion ros;
  ros.w = xpp.w();
  ros.x = xpp.x();
  ros.y = xpp.y();
  ros.z = xpp.z();

  return ros;
}

static xpp_msgs::State6d
ToRos(const State3d& xpp)
{
  xpp_msgs::State6d msg;

  msg.pose.position = ToRos<geometry_msgs::Point>(xpp.lin.p_);
  msg.twist.linear  = ToRos<geometry_msgs::Vector3>(xpp.lin.v_);
  msg.accel.linear  = ToRos<geometry_msgs::Vector3>(xpp.lin.a_);

  msg.pose.orientation = ToRos(xpp.ang.q);
  msg.twist.angular    = ToRos<geometry_msgs::Vector3>(xpp.ang.w);
  msg.accel.angular    = ToRos<geometry_msgs::Vector3>(xpp.ang.wd);

  return msg;
}

static State3d
ToXpp(const xpp_msgs::State6d& ros)
{
  State3d xpp;

  xpp.lin.p_ = ToXpp(ros.pose.position);
  xpp.lin.v_ = ToXpp(ros.twist.linear);
  xpp.lin.a_ = ToXpp(ros.accel.linear);

  xpp.ang.q = ToXpp(ros.pose.orientation);
  xpp.ang.w = ToXpp(ros.twist.angular);
  xpp.ang.wd = ToXpp(ros.accel.angular);

  return xpp;
}

static xpp_msgs::RobotStateCartesian
ToRos(const RobotStateCartesian& xpp)
{
  xpp_msgs::RobotStateCartesian ros;

  ros.base            = ToRos(xpp.base_);
  ros.time_from_start = ros::Duration(xpp.t_global_);

  for (auto ee : xpp.ee_contact_.GetEEsOrdered()) {
    ros.ee_motion. push_back(ToRos(xpp.ee_motion_.at(ee)));
    ros.ee_contact.push_back(xpp.ee_contact_.at(ee));
    ros.ee_forces. push_back(ToRos<geometry_msgs::Vector3>(xpp.ee_forces_.at(ee)));
  }

  return ros;
}

static RobotStateCartesian
ToXpp(const xpp_msgs::RobotStateCartesian& ros)
{

  int n_ee = ros.ee_motion.size();
  RobotStateCartesian xpp(n_ee);

  xpp.base_     = ToXpp(ros.base);
  xpp.t_global_ = ros.time_from_start.toSec();

  for (auto ee : xpp.ee_contact_.GetEEsOrdered()) {
    xpp.ee_motion_.at(ee)  = ToXpp(ros.ee_motion.at(ee));
    xpp.ee_contact_.at(ee) = ros.ee_contact.at(ee);
    xpp.ee_forces_.at(ee)  = ToXpp(ros.ee_forces.at(ee));
  }

  return xpp;
}

static xpp_msgs::RobotStateCartesianTrajectory
ToRos(const std::vector<RobotStateCartesian>& xpp)
{
  xpp_msgs::RobotStateCartesianTrajectory msg;

  for (const auto state : xpp) {
    auto state_msg = ToRos(state);
    msg.points.push_back(state_msg);
  }

  return msg;
}

static std::vector<RobotStateCartesian>
ToXpp(const xpp_msgs::RobotStateCartesianTrajectory& ros)
{
  std::vector<RobotStateCartesian> xpp_vec;

  for (const auto ros_state : ros.points) {
    auto xpp = ToXpp(ros_state);
    xpp_vec.push_back(xpp);
  }

  return xpp_vec;
}

};

} // namespace xpp

#endif /* XPP_ROS_CONVERSIONS_H_ */
