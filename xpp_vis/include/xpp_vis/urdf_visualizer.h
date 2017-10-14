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


#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_states/joints.h>

namespace xpp {

class UrdfVisualizer {
public:
  using URDFName             = std::string;
//  using UrdfJointNames       = std::map<JointID, URDFName>;
  using UrdfnameToJointAngle = std::map<URDFName, double>;

  UrdfVisualizer(const std::vector<URDFName>& joint_names_in_urdf,
                 const URDFName& base_joint_in_urdf,
                 const std::string& urdf_name,
                 const std::string& rviz_fixed_frame,
                 const std::string& state_msg_name,
                 const std::string& tf_prefix = "");
  virtual ~UrdfVisualizer();

private:
  ros::Subscriber state_sub_des_; /// gets joint states, floating base and stance estimation
  tf::TransformBroadcaster broadcaster;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_state_publisher;

  void StateCallback(const xpp_msgs::RobotStateJoint& msg);

  UrdfnameToJointAngle AssignAngleToURDFJointName(const sensor_msgs::JointState &msg) const;
  geometry_msgs::TransformStamped GetBaseFromRos(const ::ros::Time& stamp, const geometry_msgs::Pose &msg) const;

  std::vector<URDFName> joint_names_in_urdf_;
  URDFName base_joint_in_urdf_;

//  UrdfJointNames urdf_joint_names_;
  std::string state_msg_name_;
  std::string rviz_fixed_frame_;
  std::string tf_prefix_;
};

} // namespace xpp

#endif /* ROBOT_VIS_BASE_H_ */
