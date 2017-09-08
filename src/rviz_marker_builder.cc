/**
 @file    rviz_marker_builder.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Defines a class that builds rviz markers
 */

#include <xpp/rviz_marker_builder.h>

#include <xpp/ros/ros_conversions.h>
#include <xpp/height_map.h>

namespace xpp {

RvizMarkerBuilder::RvizMarkerBuilder()
{
  // define a few colors
  red.a = green.a = blue.a = white.a = brown.a = yellow.a = purple.a = black.a = 1.0;

  black.r  =           black.g  =           black.b  = 0.1;
  gray.r  =            gray.g  =            gray.b  = 0.7;
  red.r    = 1.0;      red.g    = 0.0;      red.b    = 0.0;
  green.r  = 0.0;      green.g  = 150./255; green.b  = 76./255;
  blue.r   = 0.0;      blue.g   = 102./255; blue.b   = 204./255;
  brown.r  = 122./255; brown.g  = 61./255;  brown.b  = 0.0;
  white.b  =           white.g  =           white.r  = 1.0;
  yellow.r = 204./255; yellow.g = 204./255; yellow.b = 0.0;
  purple.r = 72./255;  purple.g = 61./255;  purple.b = 139./255;

  terrain_ = opt::HeightMap::MakeTerrain(opt::HeightMap::FlatID);
}

void
RvizMarkerBuilder::SetOptimizationParameters (const xpp_msgs::OptParameters& msg)
{
  params_ = msg;
}

//RvizMarkerBuilder::MarkerArray
//RvizMarkerBuilder::BuildTrajectoryMarkers (const RobotCartTraj& traj) const
//{
//  MarkerArray msg;
//  int id = BuildStateMarkers(traj.front()).markers.back().id+1; // to not interfere with state
//  int i=0;
//  for (const auto& state : traj) {
//
//    // only plot every second state
//    if (i++%2 != 0)
//      continue;
//
//    MarkerArray msg_state = BuildStateMarkers(state);
//
//    // adapt some parameters
//    for (Marker m : msg_state.markers) {
//
//      m.color.a = 0.2; // make slightly transparent
//
//      if (m.ns == "support_polygons")
//        m.color.a = 0.41/(20*m.points.size()+1); // more transparent for support triangles
//
//      if (m.type == Marker::SPHERE)
//        m.scale.x = m.scale.y = m.scale.z = 0.01;
//
//      if (m.ns == "ee_force" ||
//          m.ns == "inverted_pendulum" ||
//          m.ns == "gravity_force")
//        continue; // don't plot endeffector forces in trajectory
//
//      if (m.ns == "base_pose")
//        m.scale.x = m.scale.y = m.scale.z = 0.01;
//
//      m.id = id++;
////      m.lifetime = ::ros::Duration(100);
//      m.ns = "traj_" + m.ns;
//      msg.markers.push_back(m);
//    }
//  }
//
//  return msg;
//}

RvizMarkerBuilder::MarkerArray
RvizMarkerBuilder::BuildStateMarkers (const RobotState& state) const
{
  MarkerArray msg;

  Marker base = CreateBasePose(state.base_.lin.p_,
                               state.base_.ang.q,
                               state.ee_contact_);
  msg.markers.push_back(base);

  Marker cop = CreateCopPos(state.ee_forces_,state.GetEEPos());
  msg.markers.push_back(cop);

  MarkerVec ee_pos = CreateEEPositions(state.GetEEPos(), state.ee_contact_);
  msg.markers.insert(msg.markers.begin(), ee_pos.begin(), ee_pos.end());

  MarkerVec ee_forces = CreateEEForces(state.ee_forces_,state.GetEEPos(), state.ee_contact_);
  msg.markers.insert(msg.markers.begin(), ee_forces.begin(), ee_forces.end());

  MarkerVec rom = CreateRangeOfMotion(state.base_);
  msg.markers.insert(msg.markers.begin(), rom.begin(), rom.end());

  MarkerVec support = CreateSupportArea(state.ee_contact_,state.GetEEPos());
  msg.markers.insert(msg.markers.begin(), support.begin(), support.end());

  Marker ip = CreatePendulum(state.base_.lin.p_, state.ee_forces_,state.GetEEPos());
  msg.markers.push_back(ip);

  msg.markers.push_back(CreateGravityForce(state.base_.lin.p_));


  if (state.t_global_ < 0.01) // first state in trajectory
    trajectory_id_ = trajectory_ids_start_;

  int id = state_ids_start_; // earlier IDs filled by terrain
  for (Marker& m : msg.markers) {
    m.header.frame_id = frame_id_;

    // use unique ID that doesn't get overwritten in next state.
    if (false /*m.lifetime == ::ros::DURATION_MAX*/)
      m.id = trajectory_id_++;
    else
      m.id = id;

    id++;

    //  m.header.stamp = ::ros::Time();
    //  m.action = visualization_msgs::Marker::MODIFY;
    //  m.action = visualization_msgs::Marker::DELETE;
    //  msg.markers.push_back(m);
  }

  return msg;
}

RvizMarkerBuilder::MarkerArray
RvizMarkerBuilder::BuildTerrain (int terrain)
{
  using namespace xpp::opt;
  terrain_ = HeightMap::MakeTerrain(static_cast<opt::HeightMap::ID>(terrain));

  MarkerArray msg;

  switch (terrain) {
    case HeightMap::FlatID:      msg = BuildTerrainFlat(); break;
    case HeightMap::BlockID:     msg = BuildTerrainBlock(); break;
    case HeightMap::StairsID:    msg = BuildTerrainStairs(); break;
    case HeightMap::GapID:       msg = BuildTerrainGap(); break;
    case HeightMap::SlopeID:     msg = BuildTerrainSlope(); break;
    case HeightMap::ChimneyID:   msg = BuildTerrainChimney(); break;
    case HeightMap::ChimneyLRID: msg = BuildTerrainChimneyLR(); break;
    default: return MarkerArray(); // terrain visualization not implemented
  }

  int id = terrain_ids_start_;
  for (Marker& m : msg.markers)
    m.id = id++;

  return msg;
}

RvizMarkerBuilder::MarkerArray
RvizMarkerBuilder::BuildTerrainFlat() const
{
  MarkerArray msg;

  for (int i=0; i<terrain_ids_start_; ++i) {
    msg.markers.push_back(BuildTerrainBlock(Vector3d(), Vector3d()));
    msg.markers.back().color.a = 0.0;
  }

  // one long path
  Vector3d size_start_end(5,1,0.1);
  Vector3d center0(1.5, 0.0, -0.05-eps_);
  msg.markers.at(0) = BuildTerrainBlock(center0, size_start_end);

  return msg;
}

RvizMarkerBuilder::MarkerArray
RvizMarkerBuilder::BuildTerrainBlock() const
{
  double block_start = 1.5;
  double length_     = 1.0;
  double height_     = 0.4; // [m]


  MarkerArray msg;
  double area_width = 3.0;

  Vector3d size0(4.5,area_width,0.1);
  Vector3d center0(1.25, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size0));



  Vector3d size(length_,area_width,height_);
  Vector3d center1(size.x()/2 + block_start, 0.0, size.z()/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size));

  return msg;
}


RvizMarkerBuilder::MarkerArray
RvizMarkerBuilder::BuildTerrainStairs() const
{
  double first_step_start = 1.5;
  double height_first_step = 0.2;
  double first_step_width = 0.4;
  double width_top = 1.0;


  MarkerArray msg;
  double area_width = 3.0;

  Vector3d size0(4.5,area_width,0.1);
  Vector3d center0(1.25, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size0));


  Vector3d size(first_step_width+width_top,area_width,height_first_step);
  Vector3d center1(size.x()/2 + first_step_start, 0.0, size.z()/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size));

  double height_second_step = 0.4;
  Vector3d size2(width_top,area_width,height_second_step);
  Vector3d pos2(first_step_start+first_step_width+size2.x()/2, 0.0, size2.z()/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(pos2, size2));

  return msg;
}

RvizMarkerBuilder::MarkerArray
RvizMarkerBuilder::BuildTerrainGap() const
{
  MarkerArray msg;

  double gap_start = 1.5;
  double l_gap = 0.5;

  double lx = gap_start*2.0;
  double ly = 3.0;
  double lz = 0.5;


  Vector3d size0(4.5,1,0.1);
  Vector3d center0(1.25, 0.0, -lz-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size0));

  Vector3d size(lx,ly,lz);
  Vector3d center1(0.0, 0.0, -lz/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(center1, size));

  Vector3d pos2(l_gap + lx, 0.0, -lz/2-eps_);
  msg.markers.push_back(BuildTerrainBlock(pos2, size));

  return msg;
}

RvizMarkerBuilder::MarkerArray
RvizMarkerBuilder::BuildTerrainSlope() const
{
  MarkerArray msg;

  const double slope_start = 0.5;
  const double up_length_   = 1.0;
  const double down_length_ = 1.0;
  const double height_center = 0.5;
  const double x_down_start_ = slope_start+up_length_;
  const double x_flat_start_ = x_down_start_ + down_length_;
  const double slope = height_center/up_length_;

  double length_start_end_ = 2.0; // [m]


  Vector3d size_start_end(2,1,0.1);
  Vector3d center0(-length_start_end_/2. + slope_start, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size_start_end));


  double roll = 0.0;
  double pitch = -atan(slope);
  double yaw = 0.0;
  Eigen::Quaterniond ori =  GetQuaternionFromEulerZYX(yaw, pitch, roll);


  double lx = height_center/sin(pitch);
  double ly = 1.0;
  double lz = 0.04;
  Vector3d size(lx,ly,lz);

  // slope up
  Vector3d center1(slope_start+up_length_/2, 0.0, height_center/2-lz);
  msg.markers.push_back(BuildTerrainBlock(center1, size, ori));

  // slope_down
  Vector3d center2(x_down_start_+down_length_/2, 0.0, height_center/2-lz);
  msg.markers.push_back(BuildTerrainBlock(center2, size, ori.inverse()));

  // flat end
  Vector3d center_end(length_start_end_/2.+x_flat_start_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center_end, size_start_end));


  return msg;
}

RvizMarkerBuilder::MarkerArray
RvizMarkerBuilder::BuildTerrainChimney() const
{
  MarkerArray msg;

  const double x_start_ = 0.5;
  const double length_  = 1.0;
  const double y_start_ = 0.5; // distance to start of slope from center at z=0
  const double slope    = 3;

  double length_start_end_ = 2.0; // [m]


  double roll = atan(slope);
  double pitch = 0.0;
  double yaw = 0.0;
  Eigen::Quaterniond ori =  GetQuaternionFromEulerZYX(yaw, pitch, roll);


  double lx = length_;
  double ly = 1.0;
  double lz = 0.04;
  Vector3d size(lx,ly,lz);

  double y_length = cos(roll)*ly;
  double z_height = sin(roll)*ly;


  // start
  Vector3d size_start_end(2,1,0.1);
  Vector3d center0(-length_start_end_/2. + x_start_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size_start_end));

  // slope left
  Vector3d center1(x_start_+length_/2, y_start_+eps_, 0);
  msg.markers.push_back(BuildTerrainBlock(center1, size, ori));
  msg.markers.back().color.a = 1.0;

//  // slope_right
//  Vector3d center2(x_start_+length_/2, -y_start_-eps_, 0);
//  msg.markers.push_back(BuildTerrainBlock(center2, size, ori.inverse()));
//  msg.markers.back().color.a = 0.8;

  // flat end
  Vector3d center_end(length_start_end_/2.+x_start_+length_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center_end, size_start_end));

  return msg;
}

RvizMarkerBuilder::MarkerArray
RvizMarkerBuilder::BuildTerrainChimneyLR() const
{
  MarkerArray msg;

  const double x_start_ = 0.5;
  const double length_  = 1.0;
  const double y_start_ = 0.5; // distance to start of slope from center at z=0
  const double slope    = 2;

  double length_start_end_ = 2.0; // [m]


  double roll = atan(slope);
  double pitch = 0.0;
  double yaw = 0.0;
  Eigen::Quaterniond ori =  GetQuaternionFromEulerZYX(yaw, pitch, roll);


  double lx = length_;
  double ly = 1.0;
  double lz = 0.04;
  Vector3d size(lx,ly,lz);

  double y_length = cos(roll)*ly;
  double z_height = sin(roll)*ly;


  // start
  Vector3d size_start_end(2,1,0.1);
  Vector3d center0(-length_start_end_/2. + x_start_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center0, size_start_end));

  // slope left
  Vector3d center1(x_start_+length_/2, y_start_+eps_, 0);
  msg.markers.push_back(BuildTerrainBlock(center1, size, ori));
  msg.markers.back().color.a = 0.8;

  // slope_right
  Vector3d center2(center1.x()+length_, -y_start_-eps_, 0);
  msg.markers.push_back(BuildTerrainBlock(center2, size, ori.inverse()));
  msg.markers.back().color.a = 0.8;

  // flat end
  Vector3d center_end(length_start_end_/2.+x_start_+2*length_, 0.0, -0.05-eps_);
  msg.markers.push_back(BuildTerrainBlock(center_end, size_start_end));

  return msg;
}

RvizMarkerBuilder::Marker
RvizMarkerBuilder::BuildTerrainBlock (const Vector3d& pos,
                                      const Vector3d& edge_length,
                                      Eigen::Quaterniond ori) const
{
  Marker m = CreateBox(pos, ori, edge_length);

  m.ns = "terrain";
  m.header.frame_id = "world";
  m.color = gray;
  m.color.a = 1.0;

  return m;
}

RvizMarkerBuilder::MarkerVec
RvizMarkerBuilder::CreateEEPositions (const EEPos& ee_pos, const ContactState& in_contact) const
{
  MarkerVec vec;

  for (auto ee : ee_pos.GetEEsOrdered()) {
    Marker m = CreateSphere(ee_pos.At(ee), 0.04);
    m.ns     = "endeffector_pos";
    m.color  = GetLegColor(ee);

    if (in_contact.At(ee))
      m.lifetime = ::ros::DURATION_MAX; // keep showing footholds

    vec.push_back(m);
  }

  return vec;
}

RvizMarkerBuilder::Marker
RvizMarkerBuilder::CreateGravityForce (const Vector3d& base_pos) const
{
  double g = 9.81;
  double mass = params_.base_mass;
  Marker m = CreateForceArrow(Eigen::Vector3d(0.0, 0.0, -mass*g), base_pos);
  m.color  = red;
  m.ns     = "gravity_force";

  return m;
}

RvizMarkerBuilder::MarkerVec
RvizMarkerBuilder::CreateEEForces (const EEForces& ee_forces,
                                   const EEPos& ee_pos,
                                   const ContactState& contact_state) const
{
  MarkerVec vec;

  for (auto ee : ee_forces.GetEEsOrdered()) {
    Vector3d p = ee_pos.At(ee);
    Vector3d f = ee_forces.At(ee);


    Marker m = CreateForceArrow(-f, p);
    m.color  = red;
    m.color.a = f.sum() > 0.1? 1.0 : 0.0;
    m.ns     = "ee_force";
    vec.push_back(m);

    Vector3d n = terrain_->GetNormalizedBasis(opt::HeightMap::Normal, p.x(), p.y());
    m = CreateFrictionCone(p, n);
    m.color  = red;
    m.color.a = contact_state.At(ee)? 0.25 : 0.0;
    m.ns     = "friction_cone";
    vec.push_back(m);
  }

  return vec;
}

RvizMarkerBuilder::Marker
RvizMarkerBuilder::CreateBasePose (const Vector3d& pos,
                                  Eigen::Quaterniond ori,
                                  const ContactState& contact_state) const
{
  Vector3d edge_length(0.1, 0.05, 0.02);
  Marker m = CreateBox(pos, ori, 3*edge_length);

  m.color = black;
  for (auto ee : contact_state.GetEEsOrdered())
    if (contact_state.At(ee))
      m.color = red;

  m.ns = "base_pose";

  return m;
}

RvizMarkerBuilder::Marker
RvizMarkerBuilder::CreateCopPos (const EEForces& ee_forces,
                                 const EEPos& ee_pos) const
{
  double z_sum = 0.0;
  for (Vector3d ee : ee_forces.ToImpl())
    z_sum += ee.z();

  // only then can the Center of Pressure be calculated
  Vector3d cop = Vector3d::Zero();
  if (z_sum > 0.0) {
    for (auto ee : ee_forces.GetEEsOrdered()) {
      double p = ee_forces.At(ee).z()/z_sum;
      cop.topRows<kDim2d>() += p*ee_pos.At(ee).topRows<kDim2d>();
    }
  }

  cop.z() = terrain_->GetHeight(cop.x(), cop.y());

  Marker m = CreateSphere(cop);
  m.color = red;
  m.ns = "cop";

  return m;
}

RvizMarkerBuilder::Marker
RvizMarkerBuilder::CreatePendulum (const Vector3d& pos,
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
  m.color = black;

  double fz_sum = 0.0;
  for (Vector3d ee : ee_forces.ToImpl())
    fz_sum += ee.z();

  if (fz_sum < 1.0) // [N] flight phase
    m.color.a = 0.0; // hide marker

  return m;
}

RvizMarkerBuilder::MarkerVec
RvizMarkerBuilder::CreateRangeOfMotion (const State3d& base) const
{
  MarkerVec vec;

  auto w_R_b = base.ang.q.toRotationMatrix();

  int ee = E0;
  for (const auto& pos_B : params_.nominal_ee_pos) {
    Vector3d pos_W = base.lin.p_ + w_R_b*ros::RosConversions::RosToXpp(pos_B);

    Marker m  = CreateBox(pos_W, base.ang.q, 2*ros::RosConversions::RosToXpp(params_.ee_max_dev));
    m.color   = GetLegColor(ee++);
    m.color.a = 0.2;
    m.ns      = "range_of_motion";
    vec.push_back(m);
  }

  return vec;
}

RvizMarkerBuilder::Marker
RvizMarkerBuilder::CreateBox (const Vector3d& pos, Eigen::Quaterniond ori,
                              const Vector3d& edge_length) const
{
  Marker m;

  m.type = Marker::CUBE;
  m.pose.position    = ros::RosConversions::XppToRos<geometry_msgs::Point>(pos);
  m.pose.orientation = ros::RosConversions::XppToRos(ori);
  m.scale            = ros::RosConversions::XppToRos<geometry_msgs::Vector3>(edge_length);

  return m;
}

RvizMarkerBuilder::Marker
RvizMarkerBuilder::CreateSphere (const Vector3d& pos, double diameter) const
{
  Marker m;

  m.type = Marker::SPHERE;
  m.pose.position = ros::RosConversions::XppToRos<geometry_msgs::Point>(pos);
  m.scale.x = diameter;
  m.scale.y = diameter;
  m.scale.z = diameter;

  return m;
}

RvizMarkerBuilder::Marker
RvizMarkerBuilder::CreateForceArrow (const Vector3d& force,
                                     const Vector3d& ee_pos) const
{
  Marker m;
  m.type = Marker::ARROW;

  m.scale.x = 0.01; // shaft diameter
  m.scale.y = 0.02; // arrow-head diameter
  m.scale.z = 0.06; // arrow-head length

  double force_scale = 800; //params_.base_mass*40; // scaled by base weight
  auto start = ros::RosConversions::XppToRos<geometry_msgs::Point>(ee_pos - force/force_scale);
  m.points.push_back(start);

  auto end = ros::RosConversions::XppToRos<geometry_msgs::Point>(ee_pos);
  m.points.push_back(end);

  return m;
}

RvizMarkerBuilder::Marker
RvizMarkerBuilder::CreateFrictionCone (const Vector3d& pos,
                                       const Vector3d& normal) const
{
  Marker m;
  m.type = Marker::ARROW;

  double cone_height = 0.1; // [m]
  double friction = 0.5;

  m.scale.x = 0.00; // [shaft diameter] hide arrow shaft to generate cone
  m.scale.y = 2.0 * cone_height * terrain_->GetFrictionCoeff(); // arrow-head diameter
  m.scale.z = cone_height; // arrow head length

  auto start = ros::RosConversions::XppToRos<geometry_msgs::Point>(pos + normal);
  m.points.push_back(start);

  auto end = ros::RosConversions::XppToRos<geometry_msgs::Point>(pos);
  m.points.push_back(end);

  return m;
}

RvizMarkerBuilder::MarkerVec
RvizMarkerBuilder::CreateSupportArea (const ContactState& contact_state,
                                      const EEPos& ee_pos) const
{
  MarkerVec vec;

  Marker m;
  m.ns = "support_polygons";
  m.scale.x = m.scale.y = m.scale.z = 1.0;

  for (auto ee : contact_state.GetEEsOrdered()) {
    if (contact_state.At(ee)) { // endeffector in contact
      auto p = ros::RosConversions::XppToRos<geometry_msgs::Point>(ee_pos.At(ee));
      m.points.push_back(p);
      m.color = black;
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

geometry_msgs::PoseStamped
RvizMarkerBuilder::BuildGoalPose (double x, double y,
                                  xpp_msgs::StateLin3d orientation) const
{
  geometry_msgs::PoseStamped msg_out;
  msg_out.header.frame_id = frame_id_;

  // linear part
  msg_out.pose.position.x = x;
  msg_out.pose.position.y = y;
  msg_out.pose.position.z = terrain_->GetHeight(x,y);

  // angular part
  auto goal_ang = xpp::ros::RosConversions::RosToXpp(orientation);
  Eigen::Quaterniond q = GetQuaternionFromEulerZYX(goal_ang.p_.z(),goal_ang.p_.y(), goal_ang.p_.x());
  msg_out.pose.orientation = xpp::ros::RosConversions::XppToRos(q);

  return msg_out;
}

std_msgs::ColorRGBA
RvizMarkerBuilder::GetLegColor(int ee) const
{
  std_msgs::ColorRGBA color_leg;
  switch (ee) {
    case EndeffectorID::E1:
      color_leg = purple;
      break;
    case EndeffectorID::E3:
      color_leg = green;
      break;
    case EndeffectorID::E0:
      color_leg = blue;
      break;
    case EndeffectorID::E2:
      color_leg = brown;
      break;
    default:
      break;
  }

  return color_leg;
}

//RvizMarkerBuilder::Marker
//RvizMarkerBuilder::VisualizeGoal (const Vector3d& pos) const
//{
//  Marker m = CreateSphere(pos, 0.035);
//  m.color           = black;
//  m.ns              = "goal";
//  m.header.frame_id = "world";
//  m.id = 0;
//
//  return m;
//}

//void
//MarkerArrayBuilder::AddEllipse(visualization_msgs::MarkerArray& msg,
//                               double center_x, double center_y,
//                               double width_x, double width_y,
//                               const std::string& rviz_namespace) const
//{
//
//  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
//
//  visualization_msgs::Marker ellipse;
//  ellipse.header.frame_id = frame_id_;
//  ellipse.header.stamp = ::ros::Time::now();
//  ellipse.id = i;
//  ellipse.type = visualization_msgs::Marker::CYLINDER;
//  ellipse.ns = rviz_namespace;
//  ellipse.action = visualization_msgs::Marker::MODIFY;
//  ellipse.pose.position.x = center_x;
//  ellipse.pose.position.y = center_y;
//  ellipse.pose.orientation.x = ellipse.pose.orientation.y = ellipse.pose.orientation.z = 0.0;
//  ellipse.pose.orientation.w = 1.0;
//  ellipse.color.b = 1.0;
//  ellipse.color.a = 0.2;
//
//  ellipse.scale.x = width_x;
//  ellipse.scale.y = width_y;
//  ellipse.scale.z = 0.01; // height of cylinder
//
//  msg.markers.push_back(ellipse);
//}

} /* namespace xpp */


