/**
@file    base_state_visualizer.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.01.2017
@brief   Extracts the pose from the current state and publishes to rviz
 */

#include <ros/ros.h>
#include <xpp/ros/topic_names.h>

#include <geometry_msgs/PoseStamped.h>
#include <xpp_msgs/RobotStateJointsTrajectory.h>

namespace xpp {

using PoseMsg        = geometry_msgs::PoseStamped;
using TrajectoryMsg  = xpp_msgs::RobotStateJointsTrajectory;

/** Draws an arrow in RVIZ to visualize push that generates initial velocity
  */
class InitialStateVisualizer {
public:
  InitialStateVisualizer() {
    ros::NodeHandle n;
    sub_  = n.subscribe(xpp_msgs::robot_trajectory_joints, 1, &InitialStateVisualizer::TrajectoryCallback, this);

    rviz_pub_  = n.advertise<PoseMsg>(xpp_msgs::init_velocity, 1);
    pose_arrow_msg_.header.frame_id = "world";
  }

private:
  void TrajectoryCallback (const TrajectoryMsg::ConstPtr& msg)
  {
    // Get very first state
    pose_arrow_msg_.header.stamp = ::ros::Time::now();
    pose_arrow_msg_.pose = msg->states.front().common.base.pose;
    ROS_DEBUG_STREAM("received current hyq state");

    // orient according to initial velocity
    auto initial_vel = msg->states.front().common.base.twist;
    double alpha = atan2( initial_vel.linear.y, initial_vel.linear.x);

    pose_arrow_msg_.pose.position.x -= cos(alpha)*0.8;
    pose_arrow_msg_.pose.position.y -= sin(alpha)*0.8;

    pose_arrow_msg_.pose.orientation.x = 0;
    pose_arrow_msg_.pose.orientation.y = 0;
    pose_arrow_msg_.pose.orientation.z = sin(alpha/2);
    pose_arrow_msg_.pose.orientation.w = cos(alpha/2);

    // publishing only the pose part to rviz
    rviz_pub_.publish(pose_arrow_msg_);
  }

  PoseMsg pose_arrow_msg_;
  ros::Subscriber sub_;
  ros::Publisher  rviz_pub_; ///< sends out an arrow showing the initial states' velocity/push-force.
};

} // namespace xpp

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "base_pose_publisher");

  xpp::InitialStateVisualizer initial_state_visualizer;
  ros::spin();

  return 1;
}

