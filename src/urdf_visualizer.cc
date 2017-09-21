/**
 @file    robotVisBase.hpp
 @author  Diego Pardo (depardo@ethz.ch) Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 24, 2015
 @brief   Implementation of robotVisBase.hpp

  Based on code provided by Alexander Winkler
 */

#include <xpp/urdf_visualizer.h>
#include <xpp/ros/topic_names.h>
#include <xpp/ros/ros_conversions.h>

namespace xpp {

UrdfVisualizer::UrdfVisualizer(const InverseKinematics& ik,
                               const UrdfJointNames& urdf_joint_names,
                               const std::string& urdf_name,
                               const std::string& fixed_frame,
                               const std::string& state_msg_name,
                               const std::string& tf_prefix)
{
  inverse_kinematics_ = ik;
  urdf_joint_names_   = urdf_joint_names;
  rviz_fixed_frame_   = fixed_frame;
  tf_prefix_ = tf_prefix;

  ::ros::NodeHandle nh;
  state_sub_des_ = nh.subscribe(state_msg_name, 1, &UrdfVisualizer::StateCallback, this);

  ROS_INFO("Subscribed to: %s", state_sub_curr_.getTopic().c_str());
  ROS_INFO("Subscribed to: %s", state_sub_des_.getTopic().c_str());

  // Load model from file
  KDL::Tree my_kdl_tree;
  urdf::Model my_urdf_model;
  bool model_ok  = my_urdf_model.initParam(urdf_name);
  if(!model_ok)
  {
    ROS_ERROR("Invalid URDF File");
    exit(EXIT_FAILURE);
  }
  ROS_INFO("URDF successfully parsed");
  kdl_parser::treeFromUrdfModel(my_urdf_model, my_kdl_tree);
  ROS_INFO("Robot tree is ready");

  // initialize the state publisher
  robot_state_publisher = std::make_shared<robot_state_publisher::RobotStatePublisher>(my_kdl_tree);
}

void
UrdfVisualizer::StateCallback(const StateMsg& msg)
{
  auto cart   = xpp::ros::RosConversions::RosToXpp(msg);
  VectorXd v = GetJointAngles(cart.base_,cart.GetEEPos());

  sensor_msgs::JointState joint_msg;
  joint_msg.position = std::vector<double>(v.data(), v.data()+v.size());

  VisualizeJoints(::ros::Time::now(), msg.base.pose, joint_msg);
}

VectorXd
UrdfVisualizer::GetJointAngles (const State3d& base_W, const EndeffectorsPos& ee_W) const
{
  // transform world -> base frame
  Eigen::Matrix3d B_R_W = base_W.ang.q.normalized().toRotationMatrix().inverse();

  EndeffectorsPos ee_B = ee_W;
  for (auto ee : ee_W.GetEEsOrdered())
    ee_B.At(ee) = B_R_W * (ee_W.At(ee) - base_W.lin.p_);

  return inverse_kinematics_->GetAllJointAngles(ee_B).ToVec();
}

void
UrdfVisualizer::VisualizeJoints(const ::ros::Time& stamp,
                                const geometry_msgs::Pose& baseState,
                                const sensor_msgs::JointState& jointState)
{
	auto joint_positions = GetJointsFromRos(jointState);
	auto W_X_B_message   = GetBaseFromRos(stamp, baseState);

	// Ready to publish the state
	broadcaster.sendTransform(W_X_B_message);
	robot_state_publisher->publishTransforms(joint_positions, ::ros::Time::now(),tf_prefix_);
	robot_state_publisher->publishFixedTransforms(tf_prefix_);
}

UrdfVisualizer::UrdfnameToJointAngle
UrdfVisualizer::GetJointsFromRos(const sensor_msgs::JointState &msg) const
{
  UrdfnameToJointAngle q;

  for(size_t i = 0 ; i < msg.position.size(); i++)
    q[urdf_joint_names_.at(static_cast<JointID>(i))] = msg.position[i];

  return q;
}

geometry_msgs::TransformStamped
UrdfVisualizer::GetBaseFromRos(const ::ros::Time& stamp,
                               const geometry_msgs::Pose &msg) const
{
  // Converting from joint messages to robot state
  geometry_msgs::TransformStamped W_X_B_message;
  W_X_B_message.header.stamp    = stamp;
  W_X_B_message.header.frame_id = rviz_fixed_frame_;
  // this must be the same name as the base_link in the URDF file
  W_X_B_message.child_frame_id  = tf_prefix_ + "/" + urdf_joint_names_.at(BaseJoint);

	W_X_B_message.transform.translation.x =  msg.position.x;
	W_X_B_message.transform.translation.y =  msg.position.y;
	W_X_B_message.transform.translation.z =  msg.position.z;

	W_X_B_message.transform.rotation.w = msg.orientation.w;
	W_X_B_message.transform.rotation.x = msg.orientation.x;
	W_X_B_message.transform.rotation.y = msg.orientation.y;
	W_X_B_message.transform.rotation.z = msg.orientation.z;

	return W_X_B_message;
}

UrdfVisualizer::~UrdfVisualizer()
{
}

} // namespace xpp

