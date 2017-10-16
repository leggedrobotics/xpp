
#include <xpp_vis/urdf_visualizer.h>

namespace xpp {

UrdfVisualizer::UrdfVisualizer(const std::string& urdf_name,
                               const std::vector<URDFName>& joint_names_in_urdf,
                               const URDFName& base_joint_in_urdf,
                               const std::string& fixed_frame,
                               const std::string& state_topic,
                               const std::string& tf_prefix)
{
  joint_names_in_urdf_ = joint_names_in_urdf;
  base_joint_in_urdf_  = base_joint_in_urdf;
  rviz_fixed_frame_   = fixed_frame;
  tf_prefix_ = tf_prefix;

  ::ros::NodeHandle nh;
  state_sub_des_ = nh.subscribe(state_topic, 1, &UrdfVisualizer::StateCallback, this);
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

  robot_publisher = std::make_shared<robot_state_publisher::RobotStatePublisher>(my_kdl_tree);
}

void
UrdfVisualizer::StateCallback(const xpp_msgs::RobotStateJoint& msg)
{
  auto joint_positions = AssignAngleToURDFJointName(msg.joint_state);
  auto W_X_B_message   = GetBaseFromRos(::ros::Time::now(), msg.base.pose);

  tf_broadcaster_.sendTransform(W_X_B_message);
  robot_publisher->publishTransforms(joint_positions, ::ros::Time::now(),tf_prefix_);
  robot_publisher->publishFixedTransforms(tf_prefix_);
}

UrdfVisualizer::UrdfnameToJointAngle
UrdfVisualizer::AssignAngleToURDFJointName(const sensor_msgs::JointState &msg) const
{
  UrdfnameToJointAngle q;

  for (int i=0; i<msg.position.size(); ++i)
    q[joint_names_in_urdf_.at(i)] = msg.position.at(i);

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
  W_X_B_message.child_frame_id  = tf_prefix_ + "/" + base_joint_in_urdf_;

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
