/**
@file    goal_visualizer_node_exe.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.06.2016
@brief   Visualizes a goal state in rviz that is subscribes to.
 */

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <kindr/Core>

#include <xpp_msgs/StateLin3d.h>
#include <xpp_msgs/UserCommand.h>   // listen to goal state

#include <xpp/ros/topic_names.h>
#include <xpp/ros/ros_helpers.h>


using PoseMsg = geometry_msgs::PoseStamped;

static ros::Publisher rviz_pub;
static ros::Subscriber goal_sub;

void CallbackGoal(const xpp_msgs::UserCommand& msg_in)
{
  PoseMsg msg_out;
  msg_out.header.frame_id = "world";
  msg_out.pose.position = msg_in.goal_lin.pos;

  auto goal_ang = xpp::ros::RosHelpers::RosToXpp(msg_in.goal_ang);
  kindr::EulerAnglesZyxD euler(goal_ang.p_.reverse());
  kindr::RotationQuaternionD quat(euler);
  msg_out.pose.orientation = xpp::ros::RosHelpers::XppToRos(quat.toImplementation());

  rviz_pub.publish(msg_out);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "goal_visualizer");
	ros::NodeHandle n;

	rviz_pub = n.advertise<PoseMsg>(xpp_msgs::goal_axis_rviz, 1);
	goal_sub = n.subscribe(xpp_msgs::goal_state_topic, 1, CallbackGoal);

	ros::spin();
	return 1;
}
