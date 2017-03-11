/**
 @file    rviz_marker_publisher.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Defines a class that visualizes the optimization values in rviz.
 */

#ifndef XPP_VIS_OPT_VISUALIZER
#define XPP_VIS_OPT_VISUALIZER

#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/ContactVector.h>

#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace xpp {

/** @brief Visualizes the current values of the optimization variables.
  *
  * This class is responsible for getting the state of the optimizaton
  * variables and generating ROS messages for rviz to visualize. The \c observer_
  * is responsible for supplying the interpreted optimization variables and
  * \c msg_builder_ is responsible for the generation of the ROS messages.
  */
class OptVisualizer  {
public:
  using TrajMsg         = xpp_msgs::RobotStateCartesianTrajectory;
  using ContactVecMsg   = xpp_msgs::ContactVector;

  OptVisualizer();
  virtual ~OptVisualizer ();

private:
  ::ros::Subscriber traj_sub_;
  ::ros::Subscriber contacts_sub_;
  ::ros::Publisher rviz_pub_;

  void TrajectoryCallback (const TrajMsg::ConstPtr& traj_msg);
  void ContactsCallback (const ContactVecMsg& msg);
};

} /* namespace xpp */

#endif /* XPP_VIS_OPT_VISUALIZER */
