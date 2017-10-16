#include <xpp_vis_hyq/hyq_inverse_kinematics.h>

#include <xpp_states/cartesian_declarations.h>

#include <xpp_vis_hyq/hyqleg_inverse_kinematics.h>

namespace xpp {

HyqInverseKinematics::HyqInverseKinematics ()
{
}

Joints
HyqInverseKinematics::GetAllJointAngles(const EndeffectorsPos& pos_B) const
{
  Vector3d ee_pos_H; // foothold expressed in hip frame
  std::vector<Eigen::VectorXd> q_vec;
  HyqlegInverseKinematics leg;

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

HyqInverseKinematics::~HyqInverseKinematics ()
{
}


} /* namespace xpp */


