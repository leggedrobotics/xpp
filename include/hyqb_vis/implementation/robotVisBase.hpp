/**
 @file    robotVisBase.hpp
 @author  Diego Pardo (depardo@ethz.ch) Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 24, 2015
 @brief   Implementation of robotVisBase.hpp

  Based on code provided by Alexander Winkler
 */

namespace hyqb {

template <size_t NJOINTS>
robotVisBase<NJOINTS>::robotVisBase(std::string my_robot_name, const std::array<std::string, NJOINTS>& my_robot_joints) :
	robot_joint_names(my_robot_joints),
	playbackSpeed_(1.0)
{
	std::cout<<"Calling constructor"<<std::endl;
	ros::NodeHandle nh;
}


template <size_t NJOINTS>
robotVisBase<NJOINTS>::~robotVisBase()
{
  // TODO Auto-generated destructor stub
}

template <size_t NJOINTS>
void robotVisBase<NJOINTS>::init()
{
  std::string trajectoryTopic, stateTopic;
  ros::param::param<std::string>("hyq_rviz_trajectory_topic", trajectoryTopic, std::string("/hyq_rviz_trajectory"));
  ros::param::param<std::string>("hyq_rviz_state_topic", stateTopic, std::string("/hyq_rviz_state"));
//	std::string stateTopic = getParam("state_topic", std::string("/state"));

	std::cout<<"Initializing node"<<std::endl;
	// Subscriber and callback
  ros::NodeHandle nh;
  state_sub_ = nh.subscribe(stateTopic, 1, &robotVisBase::stateCallback, this);
  traj_sub_  = nh.subscribe(trajectoryTopic, 1, &robotVisBase::trajectoryCallback, this);

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
  ROS_INFO("URDF successfully pased");
  kdl_parser::treeFromUrdfModel(my_urdf_model, my_kdl_tree);
  ROS_INFO("Robot tree is ready");

  // initialize the state publisher
  robot_state_publisher = std::make_shared<robot_state_publisher::RobotStatePublisher>(my_kdl_tree);


  setZeroState();

	geometry_msgs::TransformStamped W_X_B_message;

	W_X_B_message.header.stamp = ros::Time::now();
	W_X_B_message.header.frame_id = "world";
	W_X_B_message.child_frame_id = "base_link";

	broadcaster.sendTransform(W_X_B_message);

	robot_state_publisher->publishTransforms(model_joint_positions_, ros::Time::now(),"");
	robot_state_publisher->publishFixedTransforms("");
}

template <size_t NJOINTS>
void robotVisBase<NJOINTS>::cleanup()
{
//  ROS_INFO("Good bye from %s!", name.c_str());
}


template <size_t NJOINTS>
void robotVisBase<NJOINTS>::visualizeState(const ros::Time& stamp, const geometry_msgs::Pose& baseState, const sensor_msgs::JointState& jointState)
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

// Callback for the subscriber
template <size_t NJOINTS>
void robotVisBase<NJOINTS>::stateCallback(const hyqb_msgs::StateEstimate::ConstPtr& msg)
{
	visualizeState(ros::Time::now(), msg->base.pose, msg->joints);
}

// Callback for the subscriber
template <size_t NJOINTS>
void robotVisBase<NJOINTS>::trajectoryCallback(const TrajectoryMsg::ConstPtr& msg)
{
  ROS_INFO("Trajectory received, forwarding to rviz...");
	ros::Rate loop_rate(1.0/msg->dt.data*playbackSpeed_);

	for (size_t i=0; i<msg->states.size(); i++)
	{
		visualizeState(ros::Time::now(), msg->states[i].pose, msg->states[i].joints);

		loop_rate.sleep();
	}
}


template<size_t NJOINTS>
void robotVisBase<NJOINTS>::setZeroState()
{
	for(int i = 0 ; i < NJOINTS ; i++)
	{
		model_joint_positions_[robot_joint_names[i]] = 0;
	}
}

// updating global variables from messages
template<size_t NJOINTS>
void robotVisBase<NJOINTS>::setRobotJointsFromMessage(const sensor_msgs::JointState &msg, std::map<std::string, double>& model_joint_positions)
{
	for(size_t i = 0 ; i < NJOINTS ; i++)
	{
		// Joint values are set according to the JointState convention and shifted from the zero_pose
		model_joint_positions[robot_joint_names[i]] = msg.position[i];// - zero_state_.joints().getPosition()(base_dof + i);
	}
}

template<size_t NJOINTS>
void robotVisBase<NJOINTS>::setRobotBaseStateFromMessage(const geometry_msgs::Pose &msg, geometry_msgs::TransformStamped& W_X_B_message)
{
	W_X_B_message.transform.translation.x =  msg.position.x;
	W_X_B_message.transform.translation.y =  msg.position.y;
	W_X_B_message.transform.translation.z =  msg.position.z;

	W_X_B_message.transform.rotation.x = msg.orientation.x;
	W_X_B_message.transform.rotation.y = msg.orientation.y;
	W_X_B_message.transform.rotation.z = msg.orientation.z;
	W_X_B_message.transform.rotation.w = msg.orientation.w;
}


template<size_t NJOINTS>
void robotVisBase<NJOINTS>::setRobotJointNames(const std::array<std::string, NJOINTS>& my_robot_joints,size_t array_size)
{
	robot_joint_names = my_robot_joints;
}


} /* namespace hyqb */




