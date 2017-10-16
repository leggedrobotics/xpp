
#ifndef XPP_VIS_URDF_VISUALIZER_H_
#define XPP_VIS_URDF_VISUALIZER_H_

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

/**
 * @brief Publishes RVIZ transforms used to visualize a robot's URDF.
 *
 * This class is responsible for converting an xpp_msgs::RobotStateJoint
 * robot message and publishing the corresponding RVIZ transforms.
 */
class UrdfVisualizer {
public:
  using URDFName             = std::string;
  using UrdfnameToJointAngle = std::map<URDFName, double>;

  /**
   * @brief Constructs the visualizer for a specific URDF %urdf_name.
   * @param urdf_name  Robot description variable on the ROS parameter server.
   * @param joint_names_in_urdf  The names of the joints in the URDF file
   *        ordered in the same way as in the xpp convention
   *        (see inverse kinematics).
   * @param base_joint_in_urdf  The name of the base_link in the URDF file.
   * @param rviz_fixed_frame  The Fixed Frame name specified in RVIZ.
   * @param state_topic  The topic of the xpp_msgs::RobotStateJoint ROS message
   *        to subscribe to.
   * @param tf_prefix  In case multiple URDFS are loaded, each can be given a
   *        unique tf_prefix in RIVZ to visualize different states simultaneously.
   */
  UrdfVisualizer(const std::string& urdf_name,
                 const std::vector<URDFName>& joint_names_in_urdf,
                 const URDFName& base_joint_in_urdf,
                 const std::string& rviz_fixed_frame,
                 const std::string& state_topic,
                 const std::string& tf_prefix = "");
  virtual ~UrdfVisualizer();

private:
  ros::Subscriber state_sub_des_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_publisher;

  void StateCallback(const xpp_msgs::RobotStateJoint& msg);

  UrdfnameToJointAngle AssignAngleToURDFJointName(const sensor_msgs::JointState &msg) const;
  geometry_msgs::TransformStamped GetBaseFromRos(const ::ros::Time& stamp,
                                                 const geometry_msgs::Pose &msg) const;

  std::vector<URDFName> joint_names_in_urdf_;
  URDFName base_joint_in_urdf_;

  std::string state_msg_name_;
  std::string rviz_fixed_frame_;
  std::string tf_prefix_;
};

} // namespace xpp

#endif /* XPP_VIS_URDF_VISUALIZER_H_ */
