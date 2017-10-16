
#ifndef XPP_VIS_HYQLEG_INVERSE_KINEMATICS_H_
#define XPP_VIS_HYQLEG_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>

namespace xpp {

enum HyqJointID {HAA=0, HFE, KFE, HyqlegJointCount};

/**
 * @brief Converts a hyq foot position to joint angles.
 */
class HyqlegInverseKinematics {
public:
  using Vector3d = Eigen::Vector3d;
  enum KneeBend { Forward, Backward };

  /**
   * @brief Default c'tor initializing leg lengths with standard values.
   */
  HyqlegInverseKinematics ();
  virtual ~HyqlegInverseKinematics () {};

  /**
   * @brief Returns the joint angles to reach a Cartesian foot position.
   * @param ee_pos_H  Foot position xyz expressed in the frame attached
   * at the hip-aa (H).
   */
  Vector3d GetJointAngles(const Vector3d& ee_pos_H, KneeBend bend=Forward) const;

  /**
   * @brief Restricts the joint angles to lie inside the feasible range
   * @param q[in/out]  Current joint angle that is adapted if it exceeds
   * the specified range.
   * @param joint  Which joint (HAA, HFE, KFE) this value represents.
   */
  void EnforceLimits(double& q, HyqJointID joint) const;

private:
  Vector3d hfe_to_haa_z = Vector3d(0.0, 0.0, 0.08); //distance of HFE to HAA in z direction
  double length_thigh = 0.35; // length of upper leg
  double length_shank = 0.33; // length of lower leg
};

} /* namespace xpp */

#endif /* XPP_VIS_HYQLEG_INVERSE_KINEMATICS_H_ */
