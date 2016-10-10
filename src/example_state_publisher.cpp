/**
@file    hyqb_vis_node.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    26.03.2016
@brief
 */

#include <ros/ros.h>
#include <hyqb_vis/hyqb_vis.hpp>

// inv_kin use xpp messages for this
typedef hyqb_msgs::Trajectory TrajectoryMsg;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "example_state_publisher");

  ros::NodeHandle n;
  std::string trajectoryTopic;
  ros::param::param<std::string>("trajectory_topic", trajectoryTopic, std::string("/trajectory"));
  ros::Publisher publisher = n.advertise<TrajectoryMsg>(trajectoryTopic, 10);

  std::cout<<"Created example trajectory publisher node"<<std::endl;

  // goal position in world frame
  double goal_x = 1.0;
  while (ros::ok) {
  
    // fill the trajectory
    TrajectoryMsg traj;
    traj.dt.data = 0.01;
    int n_states = 100;
    traj.states.resize(n_states);
    for (int i=0; i<n_states; ++i) {
      traj.states.at(i).pose.position.x = goal_x*static_cast<double>(i)/n_states; // moves from zero to one meter forward
      traj.states.at(i).pose.position.y = 0;
      traj.states.at(i).pose.position.z = 0;
      traj.states.at(i).pose.orientation.w = 1;
      traj.states.at(i).joints.position.resize(12);
      traj.states.at(i).joints.position.at(2) = -1.0;
    }

    // send out the trajectory
    publisher.publish(traj);
    ros::spinOnce();
    sleep(1.0);
  }

  return 1;
}
