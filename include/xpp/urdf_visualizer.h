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

#include <xpp_msgs/RobotStateCartesian.h>

#include <xpp_states/state.h>
#include <xpp/a_inverse_kinematics.h>

namespace xpp {

class UrdfVisualizer {
public:
  using StateMsg             = xpp_msgs::RobotStateCartesian;
  using UrdfnameToJointAngle = std::map<std::string, double>;
  using UrdfJointNames       = std::map<JointID, std::string>;
  using InverseKinematics    = std::shared_ptr<AInverseKinematics>;

  UrdfVisualizer(const InverseKinematics&,
                 const UrdfJointNames&,
                 const std::string& urdf_name,
                 const std::string& rviz_fixed_frame,
                 const std::string& state_msg_name,
                 const std::string& tf_prefix = "");
  virtual ~UrdfVisualizer();

private:
  ros::Subscriber state_sub_curr_; /// gets joint states, floating base and stance estimation
  ros::Subscriber state_sub_des_; /// gets joint states, floating base and stance estimation
  tf::TransformBroadcaster broadcaster;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_state_publisher;


  void StateCallback(const StateMsg& msg);
  void VisualizeJoints(const ros::Time& stamp, const geometry_msgs::Pose& baseState,
                      const sensor_msgs::JointState& jointState);

  UrdfnameToJointAngle GetJointsFromRos(const sensor_msgs::JointState &msg) const;
  geometry_msgs::TransformStamped GetBaseFromRos(const ::ros::Time& stamp, const geometry_msgs::Pose &msg) const;

  VectorXd GetJointAngles (const State3d& base_W, const EndeffectorsPos& ee_W) const;

  InverseKinematics inverse_kinematics_;
  UrdfJointNames urdf_joint_names_;
  std::string state_msg_name_;
  std::string rviz_fixed_frame_;
  std::string tf_prefix_;
};

} // namespace xpp

#endif /* ROBOT_VIS_BASE_H_ */
