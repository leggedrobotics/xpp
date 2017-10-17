
#ifndef XPP_VIS_RVIZ_ROBOT_BUILDER_H_
#define XPP_VIS_RVIZ_ROBOT_BUILDER_H_

#include <visualization_msgs/MarkerArray.h>

#include <xpp_msgs/OptParameters.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/TerrainInfo.h>

#include <xpp_states/state.h>
#include <xpp_states/robot_state_cartesian.h>


namespace xpp {

/**
 * @brief Constructs RVIZ markers that visualize a Cartesian robot state.
 *
 * This class is responsible for converting robot states into beautiful RVIZ
 * markers. The visualize quantities such as 6D base state, endeffector
 * positions, contact state of these, endeffector forces, support areas and
 * center of pressure.
 */
class RvizRobotBuilder {
public:
  using Marker          = visualization_msgs::Marker;
  using MarkerVec       = std::vector<Marker>;
  using MarkerArray     = visualization_msgs::MarkerArray;

  using ContactState    = EndeffectorsContact;
  using EEPos           = EndeffectorsPos;
  using EEForces        = Endeffectors<Vector3d>;
  using TerrainNormals  = Endeffectors<Vector3d>;
  using RobotState      = RobotStateCartesian;

public:
  /**
   * @brief  Builds an uninitialized visualizer.
   */
  RvizRobotBuilder();
  virtual ~RvizRobotBuilder () {};

  /**
   * @brief  Constructs the RVIZ markers from the ROS message.
   * @param  msg  The ROS message describing the Cartesian robot state.
   * @return The array of RVIZ markers to be published.
   */
  MarkerArray BuildRobotState(const xpp_msgs::RobotStateCartesian& msg) const;

  /**
   * @brief  Provides additional robot info that can be used for visualization.
   * @param  msg  The ROS message.
   *
   * This additional information includes such values as the robots mass
   * (for gravity force visualization), the robots nominal endeffector
   * configuration, etc (see %OptParameters).
   */
  void SetOptimizationParameters(const xpp_msgs::OptParameters& msg);

  /**
   * @brief  Additional information that can be used for visualization.
   * @param  msg  The ROS message.
   *
   * This information is related to the terrain, such as the current terrain
   * normals at the endeffectors and friction coefficient
   * (for friction cone visualization).
   */
  void SetTerrainParameters(const xpp_msgs::TerrainInfo& msg);

private:
  // various modular functions that are stitched together to generate the
  // robot state.
  // pos_W = position expressed in world frame
  // f_W   = forces expressed in world frame.
  // c     = which leg is currently in contact with the environment.
  MarkerVec CreateEEPositions(const EEPos& pos_W,
                              const ContactState& c) const;
  MarkerVec CreateEEForces(const EEForces& f_W,
                           const EEPos& pos_W,
                           const ContactState& c) const;
  MarkerVec CreateFrictionCones(const EEPos& pos_W,
                                const ContactState& c) const;
  MarkerVec CreateSupportArea(const ContactState& c,
                              const EEPos& pos_W) const;
  MarkerVec CreateRangeOfMotion(const State3d& base) const;
  Marker    CreateGravityForce (const Vector3d& base_pos) const;
  Marker    CreateBasePose(const Vector3d& pos,
                           Eigen::Quaterniond ori,
                           const ContactState& c) const;
  Marker    CreateCopPos(const EEForces& f_W,
                         const EEPos& pos_W) const;
  Marker    CreatePendulum(const Vector3d& base_pos,
                           const EEForces& f_W,
                           const EEPos& pos_W) const;
  Marker    CreateFrictionCone(const Vector3d& pos_W,
                               const Vector3d& terrain_normal,
                               double friction_coeff) const;
  Marker    CreateForceArrow(const Vector3d& f,
                             const Vector3d& pos) const;
  Marker    CreateSphere(const Vector3d& pos,
                         double diameter = 0.03) const;
  Marker    CreateBox(const Vector3d& pos, Eigen::Quaterniond ori,
                      const Vector3d& edge_length) const;

  xpp_msgs::OptParameters params_msg_;
  xpp_msgs::TerrainInfo terrain_msg_;

  const std::string frame_id_ = "world";
};

} /* namespace xpp */

#endif /* XPP_VIS_RVIZ_ROBOT_BUILDER_H_ */
