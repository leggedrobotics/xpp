/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef XPP_VIS_RVIZ_ROBOT_BUILDER_H_
#define XPP_VIS_RVIZ_ROBOT_BUILDER_H_

#include <string>
#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include <xpp_msgs/RobotParameters.h>
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
  virtual ~RvizRobotBuilder () = default;

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
  void SetRobotParameters(const xpp_msgs::RobotParameters& msg);

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

  xpp_msgs::RobotParameters params_msg_;
  xpp_msgs::TerrainInfo terrain_msg_;

  const std::string frame_id_ = "world";

  /**
   * Makes sure no artifacts remain from previous visualization.
   * @param max_size  Number of marker for this rviz vector, should be constant.
   * @param vec       The vector of rviz markers.
   */
  void FillWithInvisible(int max_size, MarkerVec& vec) const;
  const static int max_ee_ = 10; // maximum number of endeffectors
};

} /* namespace xpp */

#endif /* XPP_VIS_RVIZ_ROBOT_BUILDER_H_ */
