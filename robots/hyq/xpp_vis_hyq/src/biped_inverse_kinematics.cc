#include <xpp_vis_hyq/biped_inverse_kinematics.h>

#include <cmath>
#include <iostream>

#include <xpp_states/endeffectors.h>
#include <xpp_states/cartesian_declarations.h>

#include <xpp_vis_hyq/hyqleg_inverse_kinematics.h>

namespace xpp {

BipedInverseKinematics::BipedInverseKinematics ()
{
}

Joints
BipedInverseKinematics::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  HyqlegInverseKinematics leg;
  std::vector<Eigen::VectorXd> q_vec;

  q_vec.push_back(leg.GetJointAngles(x_B.At(E0) + Vector3d(0.0, -0.1, 0.15)));
  if (x_B.GetCount() > 1)
    q_vec.push_back(leg.GetJointAngles(x_B.At(E1) + Vector3d(0.0,  0.1, 0.15)));

  return Joints(q_vec);
}

BipedInverseKinematics::~BipedInverseKinematics ()
{
}

} /* namespace xpp */


