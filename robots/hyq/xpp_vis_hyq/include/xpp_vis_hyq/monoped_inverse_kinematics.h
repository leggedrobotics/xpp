#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#include <xpp_vis/a_inverse_kinematics.h>

namespace xpp {

/** @brief The SL implementation of the inverse kinematics */
class MonopedInverseKinematics : public AInverseKinematics {
public:
  MonopedInverseKinematics();
	virtual ~MonopedInverseKinematics();

	/** @brief Returns the joint angles to reach for a specific endeffector position
	  *
	  * @param pos_b the 3d position of the endeffector expressed in the base frame
	  * @return the joints angles of the robot
	  */
	Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const override;
};

} /* namespace xpp  */

#endif /* INVERSEKINEMATICS_H_ */
