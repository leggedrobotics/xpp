
#include <xpp_vis_hyq/inverse_kinematics_hyq1.h>

#include <cmath>
#include <iostream>

namespace xpp {

InverseKinematicsHyq1::InverseKinematicsHyq1 ()
{
}

Joints
InverseKinematicsHyq1::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  Eigen::Vector3d offset_base_to_hip(0.0, 0.0, 0.15);
  Eigen::VectorXd q0 = leg.GetJointAngles(x_B.At(E0) + offset_base_to_hip);

  return Joints({q0});
}

InverseKinematicsHyq1::~InverseKinematicsHyq1 ()
{
}

} /* namespace xpp */


