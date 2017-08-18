#include <xpp/biped/biped_inverse_kinematics.h>

#include <cmath>
#include <iostream>

#include <xpp/endeffectors.h>
#include <xpp/cartesian_declarations.h>
#include <xpp/mono/hyqleg_inverse_kinematics.h>

namespace xpp {
namespace biped {


BipedInverseKinematics::BipedInverseKinematics ()
{
}

Joints
BipedInverseKinematics::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  mono::HyqlegInverseKinematics leg;

  Eigen::VectorXd q0 = leg.GetJointAngles(x_B.At(E0) + Vector3d(0.0, -0.1, 0.15));
  Eigen::VectorXd q1 = leg.GetJointAngles(x_B.At(E1) + Vector3d(0.0,  0.1, 0.15));

  return Joints({q0,q1});
}

BipedInverseKinematics::~BipedInverseKinematics ()
{
}

} /* namespace biped */
} /* namespace xpp */


