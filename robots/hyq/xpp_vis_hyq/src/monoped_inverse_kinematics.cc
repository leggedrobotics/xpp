
#include <xpp_vis_hyq/monoped_inverse_kinematics.h>

#include <cmath>
#include <iostream>

#include <xpp_vis_hyq/hyqleg_inverse_kinematics.h>

namespace xpp {

MonopedInverseKinematics::MonopedInverseKinematics ()
{
}

Joints
MonopedInverseKinematics::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  HyqlegInverseKinematics leg;

  Eigen::Vector3d offset_base_to_hip(0.0, 0.0, 0.15);
  Eigen::VectorXd q0 = leg.GetJointAngles(x_B.At(E0) + offset_base_to_hip);

  return Joints({q0});
}

MonopedInverseKinematics::~MonopedInverseKinematics ()
{
}

} /* namespace xpp */


