#ifndef XPP_BIPED_BIPED_INVERSEKINEMATICS_H_
#define XPP_BIPED_BIPED_INVERSEKINEMATICS_H_

#include <xpp_vis/a_inverse_kinematics.h>

namespace xpp {

/** @brief The SL implementation of the inverse kinematics */
class BipedInverseKinematics : public AInverseKinematics {
public:
  BipedInverseKinematics();
	virtual ~BipedInverseKinematics();

	/** @brief Returns the joint angles to reach for a specific endeffector position
	  *
	  * @param pos_b the 3d position of the endeffector expressed in the base frame
	  * @return the joints angles of the robot
	  */
	Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const override;
};

} /* namespace xpp  */

#endif /* XPP_BIPED_BIPED_INVERSEKINEMATICS_H_ */
