/**
@file    hyqb_vis_node.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.03.2016
@brief
 */

#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include <gazebo_msgs/LinkState.h>

using ModelStateMsg    = gazebo_msgs::ModelState;
using LinkStatesMsg    = gazebo_msgs::LinkState;
using ModelStatesMsg   = gazebo_msgs::ModelStates;
//using SetModelStateMsg = gazebo_msgs::SetModelState;
using Offset           = geometry_msgs::Point;

static geometry_msgs::Pose hyq_pose;
static geometry_msgs::Twist hyq_twist;

void CurrentStateCallback (const ModelStatesMsg& msg)
{
  int idx = 0;
  for (int i=0; i<msg.name.size(); ++i)
    if (msg.name.at(i) == "hyq") {
      hyq_pose = msg.pose.at(i);
      hyq_twist = msg.twist.at(i);
    }

  ROS_INFO_STREAM("received current state z:" << hyq_pose.position.z);
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gazebo_state_publisher");

  ros::NodeHandle n;
  ros::Subscriber subscriber     = n.subscribe("/gazebo/model_states", 10, CurrentStateCallback);
  ros::Publisher publisher       = n.advertise<ModelStateMsg>("/gazebo/set_model_state", 1);
  ros::Publisher publisher_link  = n.advertise<LinkStatesMsg>("/gazebo/set_link_state", 1);

  std::cout<<"Created example trajectory publisher node"<<std::endl;

  // create a simulated "push" on hyq
  LinkStatesMsg msg_hyq;
  msg_hyq.link_name = "hyq::base_link";

  // create a ball to throw on hyq
  ModelStateMsg msg;
  msg.model_name = "unit_sphere_1";

  Offset offset;
  offset.x = 0.0;
  offset.y = 1.0;
  offset.z = 0.0;

  while (ros::ok()) {

    double vel = 0.0;
    std::cout << "specify push velocity: ";
    std::cin >> vel;
//    std::cin.get(); // use to pause after every iteration


//    // where to shoot from
//    msg.pose = hyq_pose;
//    msg.pose.position.x += offset.x;
//    msg.pose.position.y += offset.y;
//    msg.pose.position.z += offset.z;
//
//    //  msg.twist.linear.x = 10*(-offset.x);
//    msg.twist.linear.y = 8*(-offset.y);
//    //  msg.twist.linear.z = 10*(-offset.z);
//
//
//    // send out the trajectory
//    publisher.publish(msg);
//    ros::spinOnce();
//
//    sleep(1.0);
//    msg.pose.position.x = 10;
//    msg.twist.linear.y = 0.0;
//    publisher.publish(msg);
    ros::spinOnce();

    // generate "push" on hyq
    msg_hyq.pose  = hyq_pose;
    msg_hyq.twist = hyq_twist;
    msg_hyq.twist.linear.y = hyq_twist.linear.y + vel; // m/s
    publisher_link.publish(msg_hyq);

  }

  return 1;
}
