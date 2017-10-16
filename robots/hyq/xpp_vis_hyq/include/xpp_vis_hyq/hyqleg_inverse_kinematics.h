/**
 @file    hyqleg_inverse_kinematics.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 18, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_VIS_INCLUDE_XPP_HYQLEG_INVERSE_KINEMATICS_H_
#define XPP_XPP_VIS_INCLUDE_XPP_HYQLEG_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>

#include "joints_monoped.h"

namespace xpp {

class HyqlegInverseKinematics {
public:
  using Vector3d = Eigen::Vector3d;
  enum KneeBend { Forward, Backward };

  HyqlegInverseKinematics ();
  virtual ~HyqlegInverseKinematics ();

  /**
   * @param ee_pos_H the foot position xyz expressed in the frame attached
   *                  at the hip-aa.
   */
  Vector3d GetJointAngles(const Vector3d& ee_pos_H, KneeBend bend=Forward) const;

  void EnforceLimits(double& val, MonopedJointID joint) const;

private:
  Vector3d hfe_to_haa_z = Vector3d(0.0, 0.0, 0.08); //distance of HFE to HAA in z direction
  double length_thigh = 0.35; // length of upper leg
  double length_shank = 0.33; // length of lower leg
};

} /* namespace xpp */

#endif /* XPP_XPP_VIS_INCLUDE_XPP_HYQLEG_INVERSE_KINEMATICS_H_ */
