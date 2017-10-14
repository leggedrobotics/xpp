/**
 @file    cartesian_joint_converter.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 14, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_VIS_INCLUDE_XPP_VIS_CARTESIAN_JOINT_CONVERTER_H_
#define XPP_XPP_VIS_INCLUDE_XPP_VIS_CARTESIAN_JOINT_CONVERTER_H_

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <xpp_msgs/RobotStateCartesian.h>

#include "a_inverse_kinematics.h"

namespace xpp {

class CartesianJointConverter {
public:
  CartesianJointConverter (const  AInverseKinematics::Ptr&,
                           const std::string& cart_topic,
                           const std::string& joint_topic);
  virtual ~CartesianJointConverter ();

private:
  void StateCallback(const xpp_msgs::RobotStateCartesian& msg);


  ros::Subscriber cart_state_sub_;
  ros::Publisher  joint_state_pub_;

  AInverseKinematics::Ptr inverse_kinematics_;
};

} /* namespace xpp */

#endif /* XPP_XPP_VIS_INCLUDE_XPP_VIS_CARTESIAN_JOINT_CONVERTER_H_ */
