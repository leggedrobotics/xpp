/**
 @file    marker_array_builder.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 31, 2016
 @brief   Declares a class that builds rviz markers
 */

#ifndef XPP_OPT_INCLUDE_MARKER_ARRAY_BUILDER_H_
#define XPP_OPT_INCLUDE_MARKER_ARRAY_BUILDER_H_

#include <xpp/robot_state_cartesian.h>
#include <visualization_msgs/MarkerArray.h>
#include <functional> //std::function

namespace xpp {

/** @brief Builds ROS marker array messages that can be visualized in rviz.
  *
  * This class is responsible for converting trajectories and footholds into
  * beautiful rviz markers. It knows nothing about trajectory optimization or
  * optimization variables.
  */
class MarkerArrayBuilder {
public:
  using ContactVec      = std::vector<Contact>;
  using Vector2d        = Eigen::Vector2d;
  using Marker          = visualization_msgs::Marker;
  using MarkerArray     = visualization_msgs::MarkerArray;
  using EEID            = EndeffectorID;

  using RobotCartTraj   = std::vector<RobotStateCartesian>;
  using FctPtr          = const std::function<Vector2d(const StateLin3d&)>;


public:
  MarkerArrayBuilder();
  virtual ~MarkerArrayBuilder () {};

public:
  RobotCartTraj robot_traj_;

  void VisualizeState(const RobotStateCartesian& state, MarkerArray& msg) const;

  void AddStart(MarkerArray& msg) const;
  void AddStartStance(MarkerArray& msg) const;
  void AddSupportPolygons(MarkerArray& msg) const;
  void AddBodyTrajectory(MarkerArray& msg) const;
  void AddZmpTrajectory(MarkerArray& msg) const;
  void AddFootholds(MarkerArray& msg) const;


  void AddPoint(MarkerArray& msg,
               const Vector2d& goal,
               std::string rviz_namespace,
               int marker_type) const;



  void AddLineStrip(MarkerArray& msg,
                    double center_x, double width_x,
                    const std::string& rviz_namespace) const;

  void AddEllipse(MarkerArray& msg,
                  double center_x, double center_y,
                  double width_x, double width_y,
                  const std::string& rviz_namespace) const;
private:
  Marker GenerateMarker(Vector2d pos, int32_t type, double size) const;
  std_msgs::ColorRGBA GetLegColor(EEID leg) const;

  void AddTrajectory(MarkerArray& msg,
                     const std::string& rviz_namespace,
                     double dt,
                     double marker_size,
                     bool is_zmp,
                     const FctPtr& Get2dValue) const;

  void AddFootholds(MarkerArray& msg,
                    const ContactVec& H_footholds,
                    const std::string& rviz_namespace,
                    int32_t type = visualization_msgs::Marker::SPHERE,
                    double alpha = 1.0) const;

  void BuildSupportPolygon(MarkerArray& msg,
                           const ContactVec& stance_legs,
                           EEID leg_id) const;

//  void AddPendulum(MarkerArray& msg,
//                   const ComMotion&,
//                   const MotionStructure&,
//                   double walking_height,
//                   const std::string& rviz_namespace,
//                   double alpha = 1.0) const;
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_MARKER_ARRAY_BUILDER_H_ */
