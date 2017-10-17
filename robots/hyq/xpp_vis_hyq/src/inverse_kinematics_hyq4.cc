#include <xpp_vis_hyq/inverse_kinematics_hyq4.h>

#include <xpp_states/cartesian_declarations.h>

namespace xpp {

InverseKinematicsHyq4::InverseKinematicsHyq4 ()
{
}

Joints
InverseKinematicsHyq4::GetAllJointAngles(const EndeffectorsPos& pos_B) const
{
  Vector3d ee_pos_H; // foothold expressed in hip frame
  std::vector<Eigen::VectorXd> q_vec;

  for (auto ee : pos_B.GetEEsOrdered()) {

    HyqlegInverseKinematics::KneeBend bend = HyqlegInverseKinematics::Forward;

    using namespace quad;
    if (ee == kMapIDToEE.at(LF)) {
      ee_pos_H = pos_B.At(ee);
    } else if (ee == kMapIDToEE.at(RF)) {
      ee_pos_H = pos_B.At(ee).cwiseProduct(Eigen::Vector3d(1,-1,1));
    } else if (ee == kMapIDToEE.at(LH)) {
      ee_pos_H = pos_B.At(ee).cwiseProduct(Eigen::Vector3d(-1,1,1));
      bend = HyqlegInverseKinematics::Backward;
    } else if (ee == kMapIDToEE.at(RH)) {
      ee_pos_H = pos_B.At(ee).cwiseProduct(Eigen::Vector3d(-1,-1,1));
      bend = HyqlegInverseKinematics::Backward;
    } else {
      assert(false); // foot_id does not exist
    }

    ee_pos_H -= base2hip_LF_;
    q_vec.push_back(leg.GetJointAngles(ee_pos_H, bend));
  }

  return Joints(q_vec);
}

InverseKinematicsHyq4::~InverseKinematicsHyq4 ()
{
}

} /* namespace xpp */
