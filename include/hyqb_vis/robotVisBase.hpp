/**
 @file    robotVisBase.hpp
 @author  Diego Pardo (depardo@ethz.ch) and Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 24, 2015
 @brief   Base class for robot visualization using rviz

 Based of the code provided by : Alexander W. Winkler

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

#include <xpp_msgs/HyqStateJointsTrajectory.h>

namespace hyqb {

template <size_t NJOINTS>
class robotVisBase {
public:
  using TrajectoryMsg = xpp_msgs::HyqStateJointsTrajectory;
  std::map<std::string, double> model_joint_positions_;

  robotVisBase(std::string my_robot_name, const std::array<std::string, NJOINTS>& my_robot_joints);
  virtual ~robotVisBase();

  void init();
  void setRobotJointNames(const std::array<std::string, NJOINTS>& my_robot_joints,size_t array_size);

protected:
  double t_;  /// time the controller has been looping

private:

  static const size_t base_dof = 6;
  std::array<std::string, NJOINTS> robot_joint_names;

  std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_state_publisher;

  //Broadcaster for the floating-base transformation */
  tf::TransformBroadcaster broadcaster;

  //  	ros::Subscriber state_sub_; /// gets joint states, floating base and stance estimation
  ros::Subscriber traj_sub_; /// gets joint states, floating base and stance estimation

  double playbackSpeed_;

  //	  void stateCallback(const hyqb_msgs::StateEstimate::ConstPtr& msg);
  void trajectoryCallback(const TrajectoryMsg::ConstPtr& msg);

  void visualizeState(const ros::Time& stamp, const geometry_msgs::Pose& baseState, const sensor_msgs::JointState& jointState);

  void setRobotJointsFromMessage(const sensor_msgs::JointState &msg, std::map<std::string, double>& model_joint_positions);
  void setRobotBaseStateFromMessage(const geometry_msgs::Pose &msg, geometry_msgs::TransformStamped& W_X_B_message);
  void setZeroState();

  void cleanup();
};

} /* namespace hyqb */

#include "implementation/robotVisBase.hpp"

#endif /* ROBOT_VIS_BASE_H_ */
