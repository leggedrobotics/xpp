/**
 @file    trajectory_to_rosbag_converter.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 */

#include <xpp/trajectory_to_rosbag_converter.h>

#include <rosbag/bag.h>

#include <xpp/rviz_marker_builder.h>
#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>

namespace xpp {

using MarkerArrayMsg = visualization_msgs::MarkerArray;

TrajectoryToRosbagConverter::TrajectoryToRosbagConverter ()
{
  ::ros::NodeHandle n;

  traj_sub_  = n.subscribe(xpp_msgs::robot_trajectory_cart, 1,
                           &TrajectoryToRosbagConverter::TrajectoryCallback, this);

  rviz_pub_  = n.advertise<MarkerArrayMsg>(xpp_msgs::rviz_optimized, 1);
  state_pub_ = n.advertise<StateMsg>(xpp_msgs::curr_robot_state, 1);
}

TrajectoryToRosbagConverter::~TrajectoryToRosbagConverter ()
{
}

void
TrajectoryToRosbagConverter::TrajectoryCallback (const TrajMsg& traj_msg)
{
  ROS_INFO_STREAM("Saving trajectory in " << filename);
  auto traj = ros::RosHelpers::RosToXppCart(traj_msg);

  // one overall view of the entire trajecetory
  RvizMarkerBuilder msg_builder;
  MarkerArrayMsg rviz_traj_msg = msg_builder.VisualizeTrajectory(traj);
  rviz_pub_.publish(rviz_traj_msg);

  // record this to be able to pause and playback later
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);
  for (int i=0; i<traj.size(); ++i)
  {
    RobotStateCartesian state = traj.at(i);
    double t_curr = state.GetTime();
    auto timestamp = ::ros::Time(t_curr + 1e-6); // to avoid t=0.0

    StateMsg current_info_msg = traj_msg.states.at(i);
    bag.write(xpp_msgs::curr_robot_state, timestamp, current_info_msg);

    // send out out to visualizers (hyq and opt)
    state_pub_.publish(current_info_msg);

    if (i != traj.size()-1) { // not at last element
      double dt = traj.at(i+1).GetTime() - t_curr;
      ::ros::Rate(1./dt).sleep();
    }
  }

  bag.close();
}

} /* namespace xpp */


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "trajectory_rosbag_converter");

  xpp::TrajectoryToRosbagConverter node;
  ROS_INFO_STREAM("Created optimization visualizer node");
  ros::spin();

  return 1;
}
