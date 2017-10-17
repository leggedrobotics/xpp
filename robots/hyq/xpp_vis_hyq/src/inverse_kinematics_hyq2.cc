#include <xpp_vis_hyq/inverse_kinematics_hyq2.h>

#include <cmath>
#include <iostream>

#include <xpp_states/endeffector_mappings.h>


namespace xpp {

InverseKinematicsHyq2::InverseKinematicsHyq2 ()
{
}

Joints
InverseKinematicsHyq2::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  using namespace biped;
  std::vector<Eigen::VectorXd> q_vec;

  q_vec.push_back(leg.GetJointAngles(x_B.at(L) + Vector3d(0.0, -0.1, 0.15)));
  q_vec.push_back(leg.GetJointAngles(x_B.at(R) + Vector3d(0.0,  0.1, 0.15)));

  return Joints(q_vec);
}

InverseKinematicsHyq2::~InverseKinematicsHyq2 ()
{
}

} /* namespace xpp */


