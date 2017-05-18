/**
 @file    rviz_marker_builder.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Declares a class that builds rviz markers
 */

#ifndef XPP_VIS_INCLUDE_RVIZ_MARKER_BUILDER_H_
#define XPP_VIS_INCLUDE_RVIZ_MARKER_BUILDER_H_

#include <xpp/robot_state_cartesian.h>
#include <visualization_msgs/MarkerArray.h>

namespace xpp {

/** @brief Builds ROS marker array messages that can be visualized in RVIZ.
  *
  * This class is responsible for converting robot trajectories that come
  * from the optimization into beautiful RVIZ markers.
  */
class RvizMarkerBuilder {
public:
  using Marker          = visualization_msgs::Marker;
  using MarkerArray     = visualization_msgs::MarkerArray;
  using MarkerVec       = std::vector<Marker>;

  using ContactState    = RobotStateCartesian::ContactState;
  using EEPos           = RobotStateCartesian::EEPos;
  using EEForces        = RobotStateCartesian::EEForces;
  using RobotCartTraj   = std::vector<RobotStateCartesian>;

public:
  RvizMarkerBuilder();
  virtual ~RvizMarkerBuilder () {};

public:
  MarkerArray VisualizeState(const RobotStateCartesian& state) const;
  MarkerArray VisualizeTrajectory(const RobotCartTraj& traj) const;
  Marker VisualizeGoal(const Vector3d& pos) const;

private:
  // next level in the hierarchy (add color and namespace)
  MarkerVec CreateEEPositions(const EEPos& ee_pos) const;
  MarkerVec CreateEEForces(const EEForces& ee_forces, const EEPos& ee_pos) const;
  Marker    CreateBasePos(const Vector3d& pos, const ContactState& contact_state) const;
  Marker    CreateCopPos(const EEForces& ee_forces, const EEPos& ee_pos) const;
  Marker    CreatePendulum(const Vector3d& pos, const EEForces& ee_forces, const EEPos& ee_pos) const;

  // new and improved functions
  Marker CreateForceArrow(const Vector3d& force, const Vector3d& ee_pos) const;
  Marker CreateSupportArea(const ContactState& contact_state, const EEPos& ee_pos) const;
  Marker CreateSphere(const Vector3d& pos, double diameter = 0.03) const;

  std_msgs::ColorRGBA red, green, blue, white, brown, yellow, purple, black;
  std_msgs::ColorRGBA GetLegColor(EndeffectorID leg) const;
};

} /* namespace xpp */

#endif /* XPP_VIS_INCLUDE_RVIZ_MARKER_BUILDER_H_ */
