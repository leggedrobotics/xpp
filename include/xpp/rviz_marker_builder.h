/**
 @file    rviz_marker_builder.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Declares a class that builds rviz markers
 */

#ifndef XPP_VIS_INCLUDE_RVIZ_MARKER_BUILDER_H_
#define XPP_VIS_INCLUDE_RVIZ_MARKER_BUILDER_H_

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <xpp_msgs/OptParameters.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/TerrainInfo.h>
#include <xpp_msgs/StateLin3d.h>

#include <xpp_states/state.h>
#include <xpp_states/robot_state_cartesian.h>


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

  using ContactState    = EndeffectorsBool;
  using EEPos           = EndeffectorsPos;
  using EEForces        = Endeffectors<Vector3d>;
  using TerrainNormals  = Endeffectors<Vector3d>;
  using RobotState      = RobotStateCartesian;
  using RobotCartTraj   = std::vector<RobotState>;


public:
  xpp_msgs::TerrainInfo terrain_msg_;


public:
  RvizMarkerBuilder();
  virtual ~RvizMarkerBuilder () {};

  void SetOptimizationParameters(const xpp_msgs::OptParameters& msg);

public:
  geometry_msgs::PoseStamped BuildGoalPose(const geometry_msgs::Point pos,
                                           xpp_msgs::StateLin3d orientation) const;

  // spring_clean_ add default state
  MarkerArray BuildStateMarkers(const xpp_msgs::RobotStateCartesian&) const;
//  MarkerArray BuildTrajectoryMarkers(const RobotCartTraj& traj) const;

  // visualizes the terrains found in xpp/height_map.h
  // spring_clean_ move these out of file
  MarkerArray BuildTerrain(int terrain_id);
  MarkerArray BuildTerrainFlat() const;
  MarkerArray BuildTerrainBlock() const;
  MarkerArray BuildTerrainStairs() const;
  MarkerArray BuildTerrainGap() const;
  MarkerArray BuildTerrainSlope() const;
  MarkerArray BuildTerrainChimney() const;
  MarkerArray BuildTerrainChimneyLR() const;

  Marker BuildTerrainBlock(const Vector3d& pos,
                           const Vector3d& edge_length,
                           Eigen::Quaterniond ori = Eigen::Quaterniond::Identity()) const;




private:
  // next level in the hierarchy (add color and namespace)
  MarkerVec CreateEEPositions(const EEPos& ee_pos, const ContactState& contact_state) const;
  MarkerVec CreateEEForces(const EEForces& ee_forces, const EEPos& ee_pos, const ContactState& contact_state) const;
  Marker    CreateGravityForce (const Vector3d& base_pos) const;
  MarkerVec CreateRangeOfMotion(const State3d& base) const;
  Marker    CreateBasePose(const Vector3d& pos, Eigen::Quaterniond ori,
                           const ContactState& contact_state) const;
  Marker    CreateCopPos(const EEForces& ee_forces, const EEPos& ee_pos) const;
  Marker    CreatePendulum(const Vector3d& pos, const EEForces& ee_forces, const EEPos& ee_pos) const;

  // new and improved functions
  Marker    CreateFrictionCone(const Vector3d& pos, const Vector3d& normal) const;
  Marker    CreateForceArrow(const Vector3d& force, const Vector3d& ee_pos) const;
  MarkerVec CreateSupportArea(const ContactState& contact_state, const EEPos& ee_pos) const;
  Marker    CreateSphere(const Vector3d& pos, double diameter = 0.03) const;
  Marker    CreateBox(const Vector3d& pos, Eigen::Quaterniond ori, const Vector3d& edge_length) const;

  std_msgs::ColorRGBA GetLegColor(int leg) const;


  std_msgs::ColorRGBA red, green, blue, dblue, white, brown, yellow, purple, black, gray, wheat;

  xpp_msgs::OptParameters params_;
  double eps_ = 0.02; // lowering of terrain
//  opt::HeightMap::Ptr terrain_;
  const int state_ids_start_ = 10;
  const int terrain_ids_start_ = 50;
  const int trajectory_ids_start_ = 70;
  mutable int trajectory_id_;

  const std::string frame_id_ = "world";
};

} /* namespace xpp */

#endif /* XPP_VIS_INCLUDE_RVIZ_MARKER_BUILDER_H_ */
