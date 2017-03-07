/**
 @file    robotVisBase.hpp
 @author  Diego Pardo (depardo@ethz.ch) Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 24, 2015
 @brief   Implementation of robotVisBase.hpp

  Based on code provided by Alexander Winkler
 */

#include <xpp/ros/topic_names.h>
#include <xpp/vis/robot_rviz_visualizer.h>

namespace xpp {
namespace vis {

RobotRvizVisualizer::RobotRvizVisualizer(std::string my_robot_name)
{
  playbackSpeed_ = 1.0;
}

RobotRvizVisualizer::~RobotRvizVisualizer()
{
}

void
RobotRvizVisualizer::init()
{
  ros::NodeHandle nh;
  state_sub_ = nh.subscribe(xpp_msgs::curr_robot_state, 1, &RobotRvizVisualizer::stateCallback, this);
  traj_sub_  = nh.subscribe(xpp_msgs::robot_trajectory_joints, 1, &RobotRvizVisualizer::trajectoryCallback, this);

  curr_state_pub_ = nh.advertise<CurrentInfoMsg>(xpp_msgs::curr_robot_state, 1);

  ROS_INFO("Subscribed to: %s", state_sub_.getTopic().c_str());
  ROS_INFO("Subscribed to: %s", traj_sub_.getTopic().c_str());

  // Load model from file
  KDL::Tree my_kdl_tree;
  urdf::Model my_urdf_model;
  bool model_ok  = my_urdf_model.initParam("hyq_rviz_urdf_robot_description");
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

	geometry_msgs::TransformStamped W_X_B_message;

	W_X_B_message.header.stamp = ros::Time::now();
	W_X_B_message.header.frame_id = "world";
	W_X_B_message.child_frame_id = "base_link";

	broadcaster.sendTransform(W_X_B_message);

	robot_state_publisher->publishTransforms(model_joint_positions_, ros::Time::now(),"");
	robot_state_publisher->publishFixedTransforms("");
}

void
RobotRvizVisualizer::visualizeState(const ros::Time& stamp, const geometry_msgs::Pose& baseState, const sensor_msgs::JointState& jointState)
{
	// Converting from joint messages to robot state
	geometry_msgs::TransformStamped W_X_B_message;
	W_X_B_message.header.stamp = stamp;
	W_X_B_message.header.frame_id = "world";
	W_X_B_message.child_frame_id = "base_link";

	setRobotJointsFromMessage(jointState, model_joint_positions_);
	setRobotBaseStateFromMessage(baseState, W_X_B_message);

	// Ready to publish the state
	broadcaster.sendTransform(W_X_B_message);
	robot_state_publisher->publishTransforms(model_joint_positions_, ros::Time::now(),"");
	robot_state_publisher->publishFixedTransforms("");
}


void
RobotRvizVisualizer::trajectoryCallback(const TrajectoryMsg::ConstPtr& msg)
{
  ROS_INFO("Trajectory received, forwarding to rviz...");
  double dt = 0.004; //[s]
	ros::Rate loop_rate(1.0/dt*playbackSpeed_);

	for (size_t i=0; i<msg->states.size(); i++)
	{
		visualizeState(ros::Time::now(), msg->states[i].base.pose, msg->states[i].joints);
		if (get_curr_from_vis_)
		  curr_state_pub_.publish(msg->states[i]);

		loop_rate.sleep();
	}
}

void
RobotRvizVisualizer::stateCallback(const CurrentInfoMsg::ConstPtr& msg)
{
  visualizeState(ros::Time::now(), msg->state.base.pose, msg->state.joints);
}

void
RobotRvizVisualizer::setRobotBaseStateFromMessage(const geometry_msgs::Pose &msg, geometry_msgs::TransformStamped& W_X_B_message)
{
	W_X_B_message.transform.translation.x =  msg.position.x;
	W_X_B_message.transform.translation.y =  msg.position.y;
	W_X_B_message.transform.translation.z =  msg.position.z;

	W_X_B_message.transform.rotation.x = msg.orientation.x;
	W_X_B_message.transform.rotation.y = msg.orientation.y;
	W_X_B_message.transform.rotation.z = msg.orientation.z;
	W_X_B_message.transform.rotation.w = msg.orientation.w;
}


} // namespace vis
} // namespace xpp




