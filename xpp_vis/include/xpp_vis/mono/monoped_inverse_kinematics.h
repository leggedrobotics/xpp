#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#include <xpp_vis/a_inverse_kinematics.h>
//#include <xpp_vis/mono/joints_monoped.h>

namespace xpp {
namespace mono {

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

//  virtual int GetJointsPerEE() const override;

private:
//	void EnforceLimits(double& val, MonopedJointID joint) const;
};

} /* namespace mono */
} /* namespace xpp  */

#endif /* INVERSEKINEMATICS_H_ */
