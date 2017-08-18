/**
 @file    hyqleg_inverse_kinematics.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 18, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_VIS_INCLUDE_XPP_HYQLEG_INVERSE_KINEMATICS_H_
#define XPP_XPP_VIS_INCLUDE_XPP_HYQLEG_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>
#include <xpp/mono/joints_monoped.h>

namespace xpp {
namespace mono {

class HyqlegInverseKinematics {
public:
  using Vector3d = Eigen::Vector3d;
  enum KneeBend { Forward, Backward };

  HyqlegInverseKinematics ();
  virtual ~HyqlegInverseKinematics ();

  /**
   * @param ee_pos_B the foot position xyz expressed in the frame attached
   *        at the hipaa.
   */
  static Vector3d GetJointAngles(const Vector3d& ee_pos_B, KneeBend bend=Forward);

  static void EnforceLimits(double& val, MonopedJointID joint);
};

} /* namespace mono */
} /* namespace xpp */

#endif /* XPP_XPP_VIS_INCLUDE_XPP_HYQLEG_INVERSE_KINEMATICS_H_ */
