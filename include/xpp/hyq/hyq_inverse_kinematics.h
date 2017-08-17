#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#include <xpp/a_inverse_kinematics.h>
#include <xpp/endeffectors4.h>

namespace xpp {
namespace hyq {

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
	JointValues GetAllJointAngles(const EndeffectorsPos& pos_b) const override;

  virtual int GetJointsPerEE() const override;

  Joints1EE GetUpperJointLimits(EEID ee) const;
  Joints1EE GetLowerJointLimits(EEID ee) const;

private:
	Joints1EE GetJointAngles(const EEPosition& pos_b, LegID ee) const;
	bool compute(LegID leg, const EEPosition& x, Eigen::Vector3d& q_bf, int &rc) const;
};

} /* namespace xpp */
} /* namespace hyq */

#endif /* INVERSEKINEMATICS_H_ */
