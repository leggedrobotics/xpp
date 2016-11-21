/**
 @file    robotVisBase.hpp
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 24, 2015
 @brief   Base class for robot visualization using rviz

          Based of the code provided by : Diego Pardo
 */

#ifndef ROBOT_VIS_BASE_H_
#define ROBOT_VIS_BASE_H_

#include <cstdlib>
#include <iostream>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <urdf/model.h>

#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>

#include <xpp_msgs/HyqStateTrajectory.h>
#include <xpp_msgs/HyqState.h>

namespace xpp {
namespace vis {

template <size_t NJOINTS>
class robotVisBase {
public:
  using TrajectoryMsg = xpp_msgs::HyqStateTrajectory;
  using HyqStateMsg   = xpp_msgs::HyqState;

  std::map<std::string, double> model_joint_positions_;

  robotVisBase(std::string my_robot_name, const std::array<std::string, NJOINTS>& my_robot_joints);
  virtual ~robotVisBase();

  void init();
  void setRobotJointNames(const std::array<std::string, NJOINTS>& my_robot_joints,size_t array_size);

private:
  static const size_t base_dof = 6;
  std::array<std::string, NJOINTS> robot_joint_names;
  double playbackSpeed_;

  ros::Subscriber state_sub_; /// gets joint states, floating base and stance estimation
  ros::Subscriber traj_sub_; /// gets joint states, floating base and stance estimation
  tf::TransformBroadcaster broadcaster;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_state_publisher;

  void stateCallback(const HyqStateMsg::ConstPtr& msg);
  void trajectoryCallback(const TrajectoryMsg::ConstPtr& msg);
  void visualizeState(const ros::Time& stamp, const geometry_msgs::Pose& baseState, const sensor_msgs::JointState& jointState);
  void setRobotJointsFromMessage(const sensor_msgs::JointState &msg, std::map<std::string, double>& model_joint_positions);
  void setRobotBaseStateFromMessage(const geometry_msgs::Pose &msg, geometry_msgs::TransformStamped& W_X_B_message);
  void setZeroState();
};

} // namespace vis
} // namespace xpp

#include "implementation/robotVisBase.hpp"

#endif /* ROBOT_VIS_BASE_H_ */
