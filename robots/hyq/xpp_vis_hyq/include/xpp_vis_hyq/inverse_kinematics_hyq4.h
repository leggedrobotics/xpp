#ifndef XPP_VIS_INVERSEKINEMATICS_HYQ4_H_
#define XPP_VIS_INVERSEKINEMATICS_HYQ4_H_

#include <xpp_vis/inverse_kinematics.h>
#include <xpp_vis_hyq/hyqleg_inverse_kinematics.h>

namespace xpp {

/**
 * @brief Inverse kinematics function for the HyQ robot.
 */
class InverseKinematicsHyq4 : public InverseKinematics {
public:
	InverseKinematicsHyq4();
	virtual ~InverseKinematicsHyq4();

  /**
   * @brief Returns joint angles to reach for a specific foot position.
   * @param pos_B  3D-position of the foot expressed in the base frame (B).
   */
	Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const override;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  int GetEECount() const override { return 4; };

private:
	Vector3d base2hip_LF_ = Vector3d(0.3735, 0.207, 0.0);
	HyqlegInverseKinematics leg;
};

} /* namespace xpp */

#endif /* XPP_VIS_INVERSEKINEMATICS_HYQ4_H_ */
