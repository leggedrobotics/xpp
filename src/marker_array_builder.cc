/**
 @file    marker_array_builder.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Defines a class that builds rviz markers
 */

#include <xpp/marker_array_builder.h>
#include <xpp/ros/ros_helpers.h>

namespace xpp {

static const std::string supp_tr_topic = "support_polygons";
static const std::string frame_id_ = "world";

static const int max_supp_polygons = 60;
static const int max_footholds = 160;
static const double max_time = 30;//s

MarkerArrayBuilder::MarkerArrayBuilder()
{
}

MarkerArrayBuilder::MarkerArray
MarkerArrayBuilder::VisualizeTrajectory (const RobotCartTraj& traj) const
{
  MarkerArray msg;
  int id = 100; // to not interfere with state
  int i=0;
  for (const auto& state : robot_traj_) {

    // only plot every tenth state
    if (i++%3 != 0)
      continue;


    MarkerArray msg_state = VisualizeState(state);

    for (Marker m : msg_state.markers) {
      m.id = id++;
      msg.markers.push_back(m);
    }

//    msg.markers.insert(msg.markers.begin(), msg_state.markers.begin(),
//                                            msg_state.markers.end());

  }

  return msg;
}

MarkerArrayBuilder::MarkerArray
MarkerArrayBuilder::VisualizeState (const RobotStateCartesian& state) const
{
  MarkerArray msg;

  Marker base = CreateBasePos(state.GetBase().lin.p_, state.GetContactState());
  msg.markers.push_back(base);

  MarkerVec ee_pos = CreateEEPositions(state.GetEEPos());
  msg.markers.insert(msg.markers.begin(), ee_pos.begin(), ee_pos.end());

  MarkerVec ee_forces = CreateEEForces(state.GetEEForces(),state.GetEEPos());
  msg.markers.insert(msg.markers.begin(), ee_forces.begin(), ee_forces.end());


  int id = 0;
  for (Marker& m : msg.markers) {
    m.header.frame_id = "world";
    m.id = id++;
    //  m.header.stamp = ::ros::Time();
    //  m.action = visualization_msgs::Marker::MODIFY;
  }

  return msg;
}

MarkerArrayBuilder::MarkerVec
MarkerArrayBuilder::CreateEEPositions (const EEPos& ee_pos) const
{
  MarkerVec vec;

  for (auto ee : ee_pos.GetEEsOrdered()) {
    Marker m = CreateSphere(ee_pos.At(ee), 0.01);
    m.color  = GetLegColor(ee);
    m.ns     = "endeffector_pos";

    vec.push_back(m);
  }

  return vec;
}

MarkerArrayBuilder::MarkerVec
MarkerArrayBuilder::CreateEEForces (const EEForces& ee_forces,
                                    const EEPos& ee_pos) const
{
  MarkerVec vec;

  for (auto ee : ee_forces.GetEEsOrdered()) {
    Marker m = CreateForceArrow(ee_forces.At(ee), ee_pos.At(ee));
    m.color  = GetLegColor(ee);
    m.ns     = "ee_force";
    vec.push_back(m);
  }

  return vec;
}

MarkerArrayBuilder::Marker
MarkerArrayBuilder::CreateBasePos (const Vector3d& pos,
                                   const ContactState& contact_state) const
{
  Marker m = CreateSphere(pos, 0.01);

  // color base like last leg in contact or black if flight phase
  m.color.r = m.color.g = m.color.b = 0.0;
  m.color.a = 1.0;
  for (auto ee : contact_state.GetEEsOrdered())
    if (contact_state.At(ee))
      m.color = GetLegColor(ee);

  m.ns = "base_pos";

  return m;
}

MarkerArrayBuilder::Marker
MarkerArrayBuilder::CreateSphere (const Vector3d& pos, double diameter) const
{
  Marker m;

  m.type = Marker::SPHERE;
  m.pose.position = ros::RosHelpers::XppToRos<geometry_msgs::Point>(pos);
  m.scale.x = diameter;
  m.scale.y = diameter;
  m.scale.z = diameter;

  return m;
}

MarkerArrayBuilder::Marker
MarkerArrayBuilder::CreateForceArrow (const Vector3d& force,
                                      const Vector3d& ee_pos) const
{
  Marker m;
  m.type = Marker::ARROW;
  m.scale.x = 0.005; // shaft diameter
  m.scale.y = 0.01; // arrow-head diameter
  m.scale.z = 0.01; // arrow-head length

  geometry_msgs::Point start, end;
  start.x = ee_pos.x();
  start.y = ee_pos.y();
  start.z = ee_pos.z();
  m.points.push_back(start);

  double force_scale = 1000;
  end.x = ee_pos.x() + force.x()/force_scale;
  end.y = ee_pos.y() + force.y()/force_scale;
  end.z = ee_pos.z() + force.z()/force_scale;
  m.points.push_back(end);

  return m;
}

MarkerArrayBuilder::Marker
MarkerArrayBuilder::CreateSupportArea (const ContactState& contact_state,
                                       const EEPos& ee_pos) const
{
  Marker m;

  m.scale.x = m.scale.y = m.scale.z = 1.0;
  m.ns = supp_tr_topic;

  for (auto ee : contact_state.GetEEsOrdered()) {
    if (contact_state.At(ee)) { // endeffector in contact
      geometry_msgs::Point p;
      p.x = ee_pos.At(ee).x();
      p.y = ee_pos.At(ee).y();
      p.z = ee_pos.At(ee).z();
      m.points.push_back(p);
    }
  }

  switch (m.points.size()) {
    case 3:
      m.type = Marker::TRIANGLE_LIST;
      break;
    case 2:
      m.type = Marker::LINE_STRIP;
      m.scale.x = 0.02;
      break;
    case 1:
      /* point not visualized */
      break;
    default:
      /* no nothing also for zero contact or all 4*/
      break;
  }

  return m;
}

void
MarkerArrayBuilder::AddStart (MarkerArray& msg) const
{
  auto start = robot_traj_.front().GetBase().lin.Get2D().p_;
  AddPoint(msg, start, "start", visualization_msgs::Marker::CYLINDER);
}

void
MarkerArrayBuilder::AddStartStance (MarkerArray& msg) const
{
  AddFootholds(msg, robot_traj_.front().GetContacts(), "start_stance", visualization_msgs::Marker::CUBE, 1.0);
}

void
MarkerArrayBuilder::AddSupportPolygons (MarkerArray& msg) const
{
  int marker_size_start = msg.markers.size();

  auto prev_contact_state = robot_traj_.front().GetContactState();
  for (const auto& state : robot_traj_) {

    if (state.GetContactState() != prev_contact_state) {

      // plot in color of last swingleg
      EEID swingleg = EEID::E0;
      for (auto ee : state.GetEndeffectors())
        if (!state.GetContactState().At(ee))
          swingleg = ee;

      BuildSupportPolygon(msg, state.GetContacts(), swingleg);
      prev_contact_state = state.GetContactState();
    }
  }

  int n_markers_inserted = msg.markers.size() - marker_size_start;

  // delete the other markers, maximum of 30 support polygons.
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
  for (uint j=n_markers_inserted; j<max_supp_polygons; ++j) {
    visualization_msgs::Marker marker;
    marker.id = i++;
    marker.ns = supp_tr_topic;
    marker.action = visualization_msgs::Marker::DELETE;
    msg.markers.push_back(marker);
  }
}

void
MarkerArrayBuilder::AddFootholds (MarkerArray& msg) const
{
  auto prev_contact_state = robot_traj_.front().GetContactState();
  ContactVec contacts;
  for (const auto& state : robot_traj_)
    if (state.GetContactState() != prev_contact_state) {
      for (auto c : state.GetContacts())
        contacts.push_back(c);
      prev_contact_state = state.GetContactState();
    }

  AddFootholds(msg, contacts, "footholds", visualization_msgs::Marker::SPHERE,
               1.0);
}

void
MarkerArrayBuilder::BuildSupportPolygon(
    visualization_msgs::MarkerArray& msg,
    const ContactVec& stance,
    EEID leg_id) const
{
//  static int i=0;
//  geometry_msgs::PolygonStamped polygon_msg;
//  polygon_msg.header.frame_id = "world";
//  polygon_msg.header.seq = i++;

  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;

  visualization_msgs::Marker m;
  m.id = i;
  m.header.frame_id = frame_id_;
  m.header.stamp = ::ros::Time();
  m.ns = supp_tr_topic; //"leg " + std::to_string(leg_id);
  m.action = visualization_msgs::Marker::MODIFY;
 //    marker.lifetime = ros::Duration(10);
  m.scale.x = m.scale.y = m.scale.z = 1.0;
  m.color = GetLegColor(leg_id);
  m.color.a = 0.15;

  static const int points_per_triangle =3;
  if (stance.size() == points_per_triangle) {
    m.type = visualization_msgs::Marker::TRIANGLE_LIST;
    for (size_t i=0; i<points_per_triangle; ++i) {
      geometry_msgs::Point point;
      point.x = stance.at(i).p.x();
      point.y = stance.at(i).p.y();
      point.z = stance.at(i).p.z();
      m.points.push_back(point);
    }
    msg.markers.push_back(m);
  }

  // if only two contact points exist, draw line
  static const int points_per_line = 2;
  if (stance.size() == points_per_line) {
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = 0.02;

    for (size_t i=0; i<points_per_line; ++i) {
      geometry_msgs::Point point;
      point.x = stance.at(i).p.x();
      point.y = stance.at(i).p.y();
      point.z = stance.at(i).p.z();
      m.points.push_back(point);
    }
    m.color.a = 0.5;
    msg.markers.push_back(m);
  }
}

void MarkerArrayBuilder::AddPoint(
    visualization_msgs::MarkerArray& msg,
    const Eigen::Vector2d& goal,
    std::string rviz_namespace,
    int marker_type) const
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;

  Marker marker;
  marker.id = i;
  marker = GenerateMarker(goal, marker_type, 0.02);
  marker.ns = rviz_namespace;
  marker.scale.z = 0.04;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  msg.markers.push_back(marker);
}

visualization_msgs::Marker
MarkerArrayBuilder::GenerateMarker(Eigen::Vector2d pos, int32_t type, double size) const
{
  visualization_msgs::Marker marker;
  marker.pose.position.x = pos.x();
  marker.pose.position.y = pos.y();
  marker.pose.position.z = 0.0;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ::ros::Time();
  marker.type = type;
  marker.action = visualization_msgs::Marker::MODIFY;
//  marker.lifetime = ::ros::Duration(0.01);
  marker.scale.x = marker.scale.y = marker.scale.z = size;
  marker.color.a = 1.0;

  return marker;
}

void
MarkerArrayBuilder::AddLineStrip(visualization_msgs::MarkerArray& msg,
                           double center_x, double depth_x,
                           const std::string& rviz_namespace) const
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = frame_id_;
  line_strip.header.stamp = ::ros::Time::now();
  line_strip.id = i;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.ns = rviz_namespace;
  line_strip.action = visualization_msgs::Marker::MODIFY;
  line_strip.pose.orientation.x = line_strip.pose.orientation.y = line_strip.pose.orientation.z = 0.0;
  line_strip.pose.orientation.w = 1.0;
  line_strip.color.b = 1.0;
  line_strip.color.a = 0.2;
  line_strip.scale.x = depth_x;
  geometry_msgs::Point p1, p2;
  p1.x = center_x;
  p1.y = -0.5;
  p1.z = 0;

  p2 = p1;
  p2.y = -p1.y;

  line_strip.points.push_back(p1);
  line_strip.points.push_back(p2);
  msg.markers.push_back(line_strip);
}

void
MarkerArrayBuilder::AddEllipse(visualization_msgs::MarkerArray& msg,
                               double center_x, double center_y,
                               double width_x, double width_y,
                               const std::string& rviz_namespace) const
{

  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;

  visualization_msgs::Marker ellipse;
  ellipse.header.frame_id = frame_id_;
  ellipse.header.stamp = ::ros::Time::now();
  ellipse.id = i;
  ellipse.type = visualization_msgs::Marker::CYLINDER;
  ellipse.ns = rviz_namespace;
  ellipse.action = visualization_msgs::Marker::MODIFY;
  ellipse.pose.position.x = center_x;
  ellipse.pose.position.y = center_y;
  ellipse.pose.orientation.x = ellipse.pose.orientation.y = ellipse.pose.orientation.z = 0.0;
  ellipse.pose.orientation.w = 1.0;
  ellipse.color.b = 1.0;
  ellipse.color.a = 0.2;

  ellipse.scale.x = width_x;
  ellipse.scale.y = width_y;
  ellipse.scale.z = 0.01; // height of cylinder

  msg.markers.push_back(ellipse);
}


void
MarkerArrayBuilder::AddBodyTrajectory (MarkerArray& msg) const
{
  double dt = 0.005;   // 0.01
  double marker_size = 0.011;
  AddTrajectory(msg, "body", dt, marker_size, false,
                [](const StateLin3d& base)
                {
    return base.Get2D().p_;
                }
  );
}

void
MarkerArrayBuilder::AddZmpTrajectory (MarkerArray& msg) const
{
  double dt = 0.025;  //0.01
  double marker_size = 0.012; // 0.008
  AddTrajectory(msg, "zmp", dt, marker_size, true,
                [](const StateLin3d& base)
                {
    // calculate zero moment point
    double z_acc   = base.a_.z();
    double height  = base.p_.z();

    Vector3d zmp = base.p_ - height/(kGravity+z_acc) * base.a_;
    Vector2d zmp_xy = zmp.topRows<kDim2d>();
    return zmp_xy;
                }
  );
}

void
MarkerArrayBuilder::AddTrajectory(visualization_msgs::MarkerArray& msg,
                                  const std::string& rviz_namespace,
                                  double dt,
                                  double marker_size,
                                  bool is_zmp,
                                  const FctPtr& Get2dValue) const
{
  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;

  double T = robot_traj_.back().GetTime() - robot_traj_.front().GetTime();
  double traj_dt = T/robot_traj_.size();

  for (double t(0.0); t < T; t+= dt) {
    visualization_msgs::Marker marker;
    auto state = robot_traj_.at(floor(t/traj_dt));

    marker = GenerateMarker(Get2dValue(state.GetBase().lin),
                            visualization_msgs::Marker::SPHERE,
                            marker_size);

    marker.id = i++;
    marker.ns = rviz_namespace;

    // plot in color of last swingleg
    marker.color.r = marker.color.g = marker.color.b = 0.5; // no swingleg
    marker.color.a = 1.0;

    for (auto ee : state.GetEndeffectors()) {
      if (state.GetContactState().At(ee)) {
        if (is_zmp) {
          marker.color.r = 1.0;
          marker.color.g = marker.color.b = 0.0;
        } else
          marker.color = GetLegColor(ee);
      }
    }


    msg.markers.push_back(marker);
  }

  // delete the other markers
  for (double t=T; t < max_time; t+= dt)
  {
    visualization_msgs::Marker marker;

    marker.id = i++;
    marker.ns = rviz_namespace;
    marker.action = visualization_msgs::Marker::DELETE;
    msg.markers.push_back(marker);
  }
}

void MarkerArrayBuilder::AddFootholds(
    visualization_msgs::MarkerArray& msg,
    const ContactVec& contacts,
    const std::string& rviz_namespace,
    int32_t type,
    double alpha) const
{

  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
  for (uint j=0; j<contacts.size(); ++j) {

    visualization_msgs::Marker marker_msg;
    marker_msg.type = type;
    marker_msg.action = visualization_msgs::Marker::MODIFY;
    marker_msg.pose.position = ros::RosHelpers::XppToRos<geometry_msgs::Point>(contacts.at(j).p);
    marker_msg.header.frame_id = frame_id_;
    marker_msg.header.stamp = ::ros::Time();
    marker_msg.ns = rviz_namespace;
    marker_msg.id = i++;
//    marker_msg.lifetime = ros::Duration(10);
    marker_msg.scale.x = 0.04;
    marker_msg.scale.y = 0.04;
    marker_msg.scale.z = 0.04;
    marker_msg.color = GetLegColor(contacts.at(j).ee);
    marker_msg.color.a = alpha;

    msg.markers.push_back(marker_msg);
  }

  // maximum of 30 steps
  for (int k=contacts.size(); k < max_footholds; k++)
  {
    visualization_msgs::Marker marker;

    marker.id = i++;
    marker.ns = rviz_namespace;
    marker.action = visualization_msgs::Marker::DELETE;
    msg.markers.push_back(marker);
  }
}

std_msgs::ColorRGBA MarkerArrayBuilder::GetLegColor(EEID ee) const
{
  // define a few colors
  std_msgs::ColorRGBA red, green, blue, white, brown, yellow, purple;
  red.a = green.a = blue.a = white.a = brown.a = yellow.a = purple.a = 1.0;

  red.r = 1.0; red.g = 0.0; red.b = 0.0;
  green.r = 0.0; green.g = 150./255; green.b = 76./255;
  blue.r = 0.0; blue.g = 102./255; blue.b = 204./255;
  brown.r = 122./255; brown.g = 61./255; brown.b = 0.0;
  white.b = white.g = white.r = 1.0;
  yellow.r = 204./255; yellow.g = 204./255; yellow.b = 0.0;
//  purple.r = 123./255; purple.g = 104./255; purple.b = 238./255;
  purple.r = 72./255; purple.g = 61./255; purple.b = 139./255;


  std_msgs::ColorRGBA color_leg;
  switch (ee) {
    case EEID::E1:
      color_leg = purple;
      break;
    case EEID::E3:
      color_leg = green;
      break;
    case EEID::E0:
      color_leg = blue;
      break;
    case EEID::E2:
      color_leg = brown;
      break;
    default:
      break;
  }

  return color_leg;
}

//void
//MarkerArrayBuilder::AddPendulum(visualization_msgs::MarkerArray& msg,
//                                const ComMotion& com_motion,
//                                const MotionStructure& motion_structure,
//                                double walking_height,
//                                const std::string& rviz_namespace,
//                                double alpha) const
//{
//
//  int i = (msg.markers.size() == 0)? 0 : msg.markers.back().id + 1;
//
//  visualization_msgs::Marker marker;
//  marker.header.frame_id = frame_id_;
//  marker.header.stamp = ::ros::Time::now();
//  marker.action = visualization_msgs::Marker::MODIFY;
//  marker.type = visualization_msgs::Marker::LINE_STRIP;
//  marker.scale.x = 0.01; // thinkness of pendulum pole
//  marker.color.a = alpha;
//
//  // everything sent here will be overwritten
//  marker.ns = rviz_namespace;
//  marker.id = 0;
//
//  double dt = 0.01;
//  for (double t(0.0); t < com_motion.GetTotalTime(); t+= dt)
//  {
//    marker.points.clear();
//    xpp::utils::StateLin2d cog_state = com_motion.GetCom(t);
//    geometry_msgs::Point point;
//    point.x = cog_state.p.x();
//    point.y = cog_state.p.y();
//    point.z = walking_height;
//    marker.points.push_back(point);
//
//    Eigen::Vector2d zmp = xpp::opt::ZeroMomentPoint::CalcZmp(cog_state.Make3D(), walking_height);
//    point.x = zmp.x();
//    point.y = zmp.y();
//    point.z = 0.0;
//    marker.points.push_back(point);
//
//
//    auto phase = motion_structure.GetCurrentPhase(t);
//    if ( !phase.IsStep() ) {
//      marker.color.r = marker.color.g = marker.color.b = 0.1;
//    } else {
//      // take color of first swingleg
//      auto swing_leg = phase.swinglegs_.front().ee;
//      marker.color = GetLegColor(swing_leg);
//
//    }
//
//    msg.markers.push_back(marker);
//  }
//
////  // delete the other markers
////  for (double t=com_motion.GetTotalTime(); t < 10.0; t+= dt)
////  {
////    visualization_msgs::Marker marker;
////
////    marker.id = i++;
////    marker.ns = rviz_namespace;
////    marker.action = visualization_msgs::Marker::DELETE;
////    msg.markers.push_back(marker);
////  }
//}

} /* namespace xpp */
