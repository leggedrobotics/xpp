#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#include <xpp/a_inverse_kinematics.h>
#include <xpp/endeffectors.h>

namespace xpp {
namespace quad {

/** @brief The SL implementation of the inverse kinematics */
class HyqInverseKinematics : public AInverseKinematics {
public:
	HyqInverseKinematics();
	virtual ~HyqInverseKinematics();

	/** @brief Returns the joint angles to reach for a specific endeffector position
	  *
	  * @param pos_b the 3d position of the endeffector expressed in the base frame
	  * @param ee the number of endeffector that the above position is referring to.
	  * @return the joints angles of the robot
	  */
	Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const override;

private:
	Vector3d base2hip_LF_ = Vector3d(0.3735, 0.207, 0.0);
};

} /* namespace quad */
} /* namespace xpp */

#endif /* INVERSEKINEMATICS_H_ */
