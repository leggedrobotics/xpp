#ifndef XPP_VIS_INVERSEKINEMATICS_HYQ1_H_
#define XPP_VIS_INVERSEKINEMATICS_HYQ1_H_

#include <xpp_vis/inverse_kinematics.h>

#include <xpp_vis_hyq/hyqleg_inverse_kinematics.h>

namespace xpp {

/**
 * @brief Inverse Kinematics for one HyQ leg attached to a brick (base).
 */
class InverseKinematicsHyq1 : public InverseKinematics {
public:
  InverseKinematicsHyq1();
	virtual ~InverseKinematicsHyq1();

	/**
	 * @brief Returns joint angles to reach for a specific foot position.
	 * @param pos_B  3D-position of the foot expressed in the base frame (B).
	 */
	Joints GetAllJointAngles(const EndeffectorsPos& pos_B) const override;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  int GetEECount() const override { return 1; };

private:
	HyqlegInverseKinematics leg;
};

} /* namespace xpp  */

#endif /* XPP_VIS_INVERSEKINEMATICS_HYQ1_H_ */
