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

#include <xpp_msgs/RobotStateCartesianTrajectory.h>

namespace xpp {

class RobotRvizVisualizer {
public:
  using TrajectoryMsg     = xpp_msgs::RobotStateCartesianTrajectory;
  using StateMsg          = xpp_msgs::RobotStateCartesian;
  using NameJointAngleMap = std::map<std::string, double>;

  RobotRvizVisualizer();
  virtual ~RobotRvizVisualizer();

protected:
  void VisualizeJoints(const ros::Time& stamp, const geometry_msgs::Pose& baseState,
                      const sensor_msgs::JointState& jointState);

private:
  ros::Subscriber state_sub_; /// gets joint states, floating base and stance estimation
  ros::Publisher curr_state_pub_; ///< publishes the current state, so can be used as simulator
  tf::TransformBroadcaster broadcaster;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_state_publisher;

  // this has to be built for publishing in rviz with the correct name of each joint
  NameJointAngleMap model_joint_positions_;

  virtual void StateCallback(const StateMsg& msg) = 0;

  virtual void SetJointsFromRos(const sensor_msgs::JointState &msg,
                                NameJointAngleMap& model_joint_positions) = 0;
  void SetBaseFromRos(const geometry_msgs::Pose &msg,
                      geometry_msgs::TransformStamped& W_X_B_message);

};

} // namespace xpp

#endif /* ROBOT_VIS_BASE_H_ */
