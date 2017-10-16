#include <xpp_vis_hyq/inverse_kinematics_hyq2.h>

#include <cmath>
#include <iostream>


namespace xpp {

InverseKinematicsHyq2::InverseKinematicsHyq2 ()
{
}

Joints
InverseKinematicsHyq2::GetAllJointAngles(const EndeffectorsPos& x_B) const
{

  std::vector<Eigen::VectorXd> q_vec;

  q_vec.push_back(leg.GetJointAngles(x_B.At(E0) + Vector3d(0.0, -0.1, 0.15)));
  if (x_B.GetCount() > 1)
    q_vec.push_back(leg.GetJointAngles(x_B.At(E1) + Vector3d(0.0,  0.1, 0.15)));

  return Joints(q_vec);
}

InverseKinematicsHyq2::~InverseKinematicsHyq2 ()
{
}

} /* namespace xpp */


