/**
@file    base_state_visualizer.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.01.2017
@brief   Extracts the pose from the current state and publishes to rviz
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <xpp_msgs/CurrentInfo.h>
#include <xpp/ros/topic_names.h>

using CurrentInfoMsg = xpp_msgs::CurrentInfo;
using PoseMsg        = geometry_msgs::PoseStamped;


class BaseStateConverter {
public:
  BaseStateConverter() {
    ros::NodeHandle n;
    sub_ = n.subscribe(xpp_msgs::curr_robot_state, 1, &BaseStateConverter::CurrentStateCallback, this);
    pub_ = n.advertise<PoseMsg>(xpp_msgs::curr_base_pose, 1);
    hyq_pose_msg_.header.frame_id = "world";
  }

private:
  void CurrentStateCallback (const CurrentInfoMsg::ConstPtr& msg)
  {
    // receiving complete body state
    hyq_pose_msg_.header.stamp = ::ros::Time::now();
    hyq_pose_msg_.pose = msg->state.base.pose;
    ROS_DEBUG_STREAM("received current hyq state");

    // publishing only the pose part
    pub_.publish(hyq_pose_msg_);
  }

  PoseMsg hyq_pose_msg_;
  ros::Subscriber sub_;
  ros::Publisher  pub_;
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "base_pose_publisher");

  BaseStateConverter converter;
  ros::spin();

  return 1;
}
