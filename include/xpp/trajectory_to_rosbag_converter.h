/**
 @file    trajectory_to_rosbag_converter.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Defines a class that visualizes the optimization values in rviz.
 */

#ifndef TRAJECTORY_TO_ROSBAG_CONVERTER_H_
#define TRAJECTORY_TO_ROSBAG_CONVERTER_H_

#include <xpp_msgs/RobotStateCartesianTrajectory.h>

#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace xpp {

/** @brief Converts the trajectory to a time-stamped ROS bag.
  *
  */
class TrajectoryToRosbagConverter  {
public:
  const std::string filename = "/home/winklera/Code/catkin_xpp/src/xpp/xpp_vis/bags/optimal_traj.bag";

  using TrajMsg  = xpp_msgs::RobotStateCartesianTrajectory;
  using StateMsg = xpp_msgs::RobotStateCartesian;

  TrajectoryToRosbagConverter();
  virtual ~TrajectoryToRosbagConverter ();

private:
  ::ros::Subscriber traj_sub_;
  ::ros::Publisher rviz_pub_;
  ::ros::Publisher state_pub_;

  /**
   * @brief saves trajectory in rosbag, adding timestamps.
   */
  void TrajectoryCallback (const TrajMsg& traj_msg);

  void StateCallback (const StateMsg& state_msg);
};

} /* namespace xpp */

#endif /* TRAJECTORY_TO_ROSBAG_CONVERTER_H_ */
