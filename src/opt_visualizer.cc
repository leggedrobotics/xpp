/**
 @file    optimization_visualizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Brief description
 */

#include <xpp/opt_visualizer.h>

#include <xpp/rviz_marker_builder.h>
#include <xpp/ros/ros_helpers.h>
#include <xpp/ros/topic_names.h>

namespace xpp {

using MarkerArrayMsg = visualization_msgs::MarkerArray;

OptVisualizer::OptVisualizer ()
{
  ::ros::NodeHandle n;

  traj_sub_      = n.subscribe(xpp_msgs::robot_trajectory_cart, 1,
                               &OptVisualizer::TrajectoryCallback, this);

  rviz_pub_      = n.advertise<MarkerArrayMsg>(xpp_msgs::rviz_optimized, 1);
}

OptVisualizer::~OptVisualizer ()
{
}

void
OptVisualizer::TrajectoryCallback (const TrajMsg::ConstPtr& traj_msg)
{
  ROS_INFO_STREAM("Received new robot trajectory");
  auto traj = ros::RosHelpers::RosToXppCart(*traj_msg);


  RvizMarkerBuilder msg_builder;
  MarkerArrayMsg msg = msg_builder.VisualizeTrajectory(traj);
  rviz_pub_.publish(msg);

  // zmp_ ugly make sure this is synched with robot visualizer
  double playbackSpeed_ = 1.0;
  double dt = 0.004; //[s]
  ::ros::Rate loop_rate(1.0/dt*playbackSpeed_);

  for (auto state : traj)
  {
    MarkerArrayMsg msg = msg_builder.VisualizeState(state);
    rviz_pub_.publish(msg);
    loop_rate.sleep();
  }
}

} /* namespace xpp */
