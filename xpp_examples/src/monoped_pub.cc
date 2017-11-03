/**
@file    monoped_pub.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    4.03.2016
@brief   Start the nlp foothold optimization server
 */

#include <ros/ros.h>

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/topic_names.h>

#include <xpp_states/convert.h>
#include <xpp_states/robot_state_cartesian.h>


using namespace xpp;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "monped_publisher_node");

  ros::NodeHandle n;
  ros::Publisher state_pub = n.advertise<xpp_msgs::RobotStateCartesian>(xpp_msgs::robot_state_desired, 1);
  ROS_INFO_STREAM("Waiting for Subscriber...");
  while(ros::ok() && state_pub.getNumSubscribers() == 0)
    ros::Rate(100).sleep();
  ROS_INFO_STREAM("Subscriber to initial state connected");


  // visualize the state of a one-legged hopper
  RobotStateCartesian hopper(1);

  // publishes a sequence of states for a total duration of T spaced 0.01s apart.
  double T = 2.0;
  double t = 0.0;
  double dt = 0.01;
  while (t < T)
  {
    // base and foot follow half a sine motion up and down
    hopper.base_.lin.p_.z() = 0.7 - 0.05*sin(2*M_PI/(2*T)*t);
    hopper.ee_motion_.at(0).p_.z() = 0.1*sin(2*M_PI/(2*T)*t);
    hopper.ee_forces_.at(0).z() = 100; // N
    hopper.ee_contact_.at(0) = true;

    state_pub.publish(Convert::ToRos(hopper));

    ros::spinOnce();
    ros::Duration(dt).sleep(); // pause loop so visualization has correct speed.
    t += dt;
  }


  return 0;
}

