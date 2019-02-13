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

#include <xpp_vis/rviz_robot_builder.h>

#include <xpp_states/convert.h>
#include <xpp_vis/rviz_colors.h>

namespace xpp {

RvizRobotBuilder::RvizRobotBuilder()
{
  terrain_msg_.friction_coeff = 0.0;
}

void
RvizRobotBuilder::SetRobotParameters (const xpp_msgs::RobotParameters& msg)
{
  params_msg_ = msg;
}

void
RvizRobotBuilder::SetTerrainParameters (const xpp_msgs::TerrainInfo& msg)
{
  terrain_msg_ = msg;
}

void
RvizRobotBuilder::FillWithInvisible(int max_size, MarkerVec& vec) const
{
  Marker invisible = vec.empty()? Marker() : vec.back();
  invisible.color.a = 0.0;
  vec.resize(max_size, invisible);
}

RvizRobotBuilder::MarkerArray
RvizRobotBuilder::BuildRobotState (const xpp_msgs::RobotStateCartesian& state_msg) const
{
  MarkerArray msg;

  auto state = Convert::ToXpp(state_msg);
  auto ee_pos = state.ee_motion_.Get(kPos);

  Marker base = CreateBasePose(state.base_.lin.p_,
                               state.base_.ang.q,
                               state.ee_contact_);
  msg.markers.push_back(base);

  MarkerVec m_ee_pos = CreateEEPositions(ee_pos, state.ee_contact_);
  FillWithInvisible(max_ee_, m_ee_pos);
  msg.markers.insert(msg.markers.begin(), m_ee_pos.begin(), m_ee_pos.end());

  MarkerVec ee_forces = CreateEEForces(state.ee_forces_, ee_pos, state.ee_contact_);
  FillWithInvisible(max_ee_, ee_forces);
  msg.markers.insert(msg.markers.begin(), ee_forces.begin(), ee_forces.end());

  MarkerVec rom = CreateRangeOfMotion(state.base_);
  FillWithInvisible(max_ee_, rom);
  msg.markers.insert(msg.markers.begin(), rom.begin(), rom.end());

  MarkerVec support = CreateSupportArea(state.ee_contact_, ee_pos);
  FillWithInvisible(max_ee_, support);
  msg.markers.insert(msg.markers.begin(), support.begin(), support.end());

  MarkerVec friction = CreateFrictionCones(ee_pos, state.ee_contact_);
  FillWithInvisible(max_ee_, friction);
  msg.markers.insert(msg.markers.begin(), friction.begin(), friction.end());

  Marker cop = CreateCopPos(state.ee_forces_, ee_pos);
  msg.markers.push_back(cop);

  Marker ip = CreatePendulum(state.base_.lin.p_, state.ee_forces_, ee_pos);
  msg.markers.push_back(ip);

  msg.markers.push_back(CreateGravityForce(state.base_.lin.p_));

  int id = 0;
  for (Marker& m : msg.markers) {
    m.header.frame_id = frame_id_;
    m.id = id++;
  }

  return msg;
}

RvizRobotBuilder::MarkerVec
RvizRobotBuilder::CreateEEPositions (const EEPos& ee_pos,
                                     const ContactState& in_contact) const
{
  MarkerVec vec;

  for (auto ee : ee_pos.GetEEsOrdered()) {
    Marker m = CreateSphere(ee_pos.at(ee), 0.04);
    m.ns     = "endeffector_pos";
    m.color  = color.blue;

    vec.push_back(m);
  }

  return vec;
}

RvizRobotBuilder::Marker
RvizRobotBuilder::CreateGravityForce (const Vector3d& base_pos) const
{
  double g = 9.81;
  double mass = params_msg_.base_mass;
  Marker m = CreateForceArrow(Eigen::Vector3d(0.0, 0.0, -mass*g), base_pos);
  m.color  = color.red;
  m.ns     = "gravity_force";

  return m;
}

RvizRobotBuilder::MarkerVec
RvizRobotBuilder::CreateEEForces (const EEForces& ee_forces,
                                  const EEPos& ee_pos,
                                  const ContactState& contact_state) const
{
  MarkerVec vec;

  for (auto ee : ee_forces.GetEEsOrdered()) {
    Vector3d p = ee_pos.at(ee);
    Vector3d f = ee_forces.at(ee);

    Marker m  = CreateForceArrow(f, p);
    m.color   = color.red;
    m.color.a = f.norm() > 0.1 ? 1.0 : 0.0;
    m.ns      = "ee_force";
    vec.push_back(m);
  }

  return vec;
}

RvizRobotBuilder::MarkerVec
RvizRobotBuilder::CreateFrictionCones (const EEPos& ee_pos,
                                       const ContactState& contact_state) const
{
  MarkerVec vec;

  // add at least one invisible to have namespace template to fill
  Marker invisible  = CreateFrictionCone(Vector3d::Zero(), Vector3d::Zero(), 0.0);
  invisible.color.a = 0.0;
  vec.push_back(invisible);

  // only draw cones if terrain_msg and robot state correspond
  double mu = terrain_msg_.friction_coeff;
  if (ee_pos.GetEECount() == terrain_msg_.surface_normals.size() && mu > 1e-3) {

    auto normals = Convert::ToXpp(terrain_msg_.surface_normals);
    for (auto ee : normals.GetEEsOrdered()) {
      Marker m;
      Vector3d n = normals.at(ee);
      m         = CreateFrictionCone(ee_pos.at(ee), -n, mu);
      m.color   = color.red;
      m.color.a = contact_state.at(ee)? 0.25 : 0.0;
      vec.push_back(m);
    }
  }

  return vec;
}

RvizRobotBuilder::Marker
RvizRobotBuilder::CreateFrictionCone (const Vector3d& pos,
                                      const Vector3d& normal,
                                      double mu) const
{
  Marker m;
  m.type = Marker::ARROW;
  m.ns   = "friction_cone";

  double cone_height = 0.1; // [m]

  m.scale.x = 0.00; // [shaft diameter] hide arrow shaft to generate cone
  m.scale.y = 2.0 * cone_height * mu; // arrow-head diameter
  m.scale.z = cone_height;            // arrow head length

  auto start = Convert::ToRos<geometry_msgs::Point>(pos + normal);
  m.points.push_back(start);

  auto end = Convert::ToRos<geometry_msgs::Point>(pos);
  m.points.push_back(end);

  return m;
}

RvizRobotBuilder::Marker
RvizRobotBuilder::CreateBasePose (const Vector3d& pos,
                                  Eigen::Quaterniond ori,
                                  const ContactState& contact_state) const
{
  Vector3d edge_length(0.1, 0.05, 0.02);
  Marker m = CreateBox(pos, ori, 3*edge_length);

  m.color = color.black;
  m.color.a = 0.8;
  for (auto ee : contact_state.GetEEsOrdered())
    if (contact_state.at(ee))
      m.color = color.black;

  m.ns = "base_pose";

  return m;
}

RvizRobotBuilder::Marker
RvizRobotBuilder::CreateCopPos (const EEForces& ee_forces,
                                const EEPos& ee_pos) const
{
  double z_sum = 0.0;
  for (Vector3d ee : ee_forces.ToImpl())
    z_sum += ee.z();

  Marker m;
  // only then can the Center of Pressure be calculated
  Vector3d cop = Vector3d::Zero();
  if (z_sum > 0.0) {
    for (auto ee : ee_forces.GetEEsOrdered()) {
      double p = ee_forces.at(ee).z()/z_sum;
      cop += p*ee_pos.at(ee);
    }
    m = CreateSphere(cop);
  } else {
    m = CreateSphere(cop, 0.001); // no CoP exists b/c flight phase
  }

  m.color = color.red;
  m.ns = "cop";

  return m;
}

RvizRobotBuilder::Marker
RvizRobotBuilder::CreatePendulum (const Vector3d& pos,
                                  const EEForces& ee_forces,
                                  const EEPos& ee_pos) const
{
  Marker m;
  m.type = Marker::LINE_STRIP;
  m.scale.x = 0.007; // thickness of pendulum pole

  geometry_msgs::Point cop = CreateCopPos(ee_forces, ee_pos).pose.position;
  geometry_msgs::Point com = CreateSphere(pos).pose.position;

  m.points.push_back(cop);
  m.points.push_back(com);

  m.ns = "inverted_pendulum";
  m.color = color.black;

  double fz_sum = 0.0;
  for (Vector3d ee : ee_forces.ToImpl())
    fz_sum += ee.z();

  if (fz_sum < 1.0) // [N] flight phase
    m.color.a = 0.0; // hide marker

  return m;
}

RvizRobotBuilder::MarkerVec
RvizRobotBuilder::CreateRangeOfMotion (const State3d& base) const
{
  MarkerVec vec;

  auto w_R_b = base.ang.q.toRotationMatrix();

  for (const auto& pos_B : params_msg_.nominal_ee_pos) {
    Vector3d pos_W = base.lin.p_ + w_R_b*Convert::ToXpp(pos_B);

    Vector3d edge_length = 2*Convert::ToXpp(params_msg_.ee_max_dev);
    Marker m  = CreateBox(pos_W, base.ang.q, edge_length);
    m.color   = color.blue;
    m.color.a = 0.2;
    m.ns      = "range_of_motion";
    vec.push_back(m);
  }

  return vec;
}

RvizRobotBuilder::Marker
RvizRobotBuilder::CreateBox (const Vector3d& pos, Eigen::Quaterniond ori,
                             const Vector3d& edge_length) const
{
  Marker m;

  m.type = Marker::CUBE;
  m.pose.position    = Convert::ToRos<geometry_msgs::Point>(pos);
  m.pose.orientation = Convert::ToRos(ori);
  m.scale            = Convert::ToRos<geometry_msgs::Vector3>(edge_length);

  return m;
}

RvizRobotBuilder::Marker
RvizRobotBuilder::CreateSphere (const Vector3d& pos, double diameter) const
{
  Marker m;

  m.type = Marker::SPHERE;
  m.pose.position = Convert::ToRos<geometry_msgs::Point>(pos);
  m.scale.x = diameter;
  m.scale.y = diameter;
  m.scale.z = diameter;

  return m;
}

RvizRobotBuilder::Marker
RvizRobotBuilder::CreateForceArrow (const Vector3d& force,
                                    const Vector3d& ee_pos) const
{
  Marker m;
  m.type = Marker::ARROW;

  m.scale.x = 0.01; // shaft diameter
  m.scale.y = 0.02; // arrow-head diameter
  m.scale.z = 0.06; // arrow-head length

  double force_scale = 800;
  Vector3d p_start = ee_pos - force/force_scale;
  auto start = Convert::ToRos<geometry_msgs::Point>(p_start);
  m.points.push_back(start);

  auto end = Convert::ToRos<geometry_msgs::Point>(ee_pos);
  m.points.push_back(end);

  return m;
}

RvizRobotBuilder::MarkerVec
RvizRobotBuilder::CreateSupportArea (const ContactState& contact_state,
                                     const EEPos& ee_pos) const
{
  MarkerVec vec;

  Marker m;
  m.ns = "support_polygons";
  m.scale.x = m.scale.y = m.scale.z = 1.0;

  for (auto ee : contact_state.GetEEsOrdered()) {
    if (contact_state.at(ee)) { // endeffector in contact
      auto p = Convert::ToRos<geometry_msgs::Point>(ee_pos.at(ee));
      m.points.push_back(p);
      m.color = color.black;
      m.color.a = 0.2;
    }
  }

  switch (m.points.size()) {
    case 4: {
      m.type = Marker::TRIANGLE_LIST;
      auto temp = m.points;

      // add two triangles to represent a square
      m.points.pop_back();
      vec.push_back(m);

      m.points = temp;
      m.points.erase(m.points.begin());
      vec.push_back(m);
      break;
    }
    case 3: {
      m.type = Marker::TRIANGLE_LIST;
      vec.push_back(m);
      vec.push_back(m);
      break;
    }
    case 2: {
      m.type = Marker::LINE_STRIP;
      m.scale.x = 0.01;
      vec.push_back(m);
      vec.push_back(m);
      break;
    }
    case 1: {
      /* just make so small that random marker can't be seen */
      m.type = Marker::CUBE; // just so same shape is specified
      m.color.a = 0.0; // hide marker
      vec.push_back(m);
      vec.push_back(m);
      break;
    }
    default:
      m.type = Marker::CUBE; // just so same shapes is specified
      m.color.a = 0.0; // hide marker
      vec.push_back(m);
      vec.push_back(m);
      break;
  }

  return vec;
}

} /* namespace xpp */
