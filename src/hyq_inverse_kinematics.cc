#include <xpp/quad/hyq_inverse_kinematics.h>

#include <xpp_states/cartesian_declarations.h>

#include <xpp/mono/hyqleg_inverse_kinematics.h>
#include <xpp/quad/joints_quadruped.h>

namespace xpp {
namespace quad {

HyqInverseKinematics::HyqInverseKinematics ()
{
}

Joints
HyqInverseKinematics::GetAllJointAngles(const EndeffectorsPos& pos_B) const
{
  Vector3d ee_pos_H; // foothold expressed in hip frame
  std::vector<Eigen::VectorXd> q_vec;
  mono::HyqlegInverseKinematics leg;

  for (auto ee : pos_B.GetEEsOrdered()) {

    std::string foot = ReverseMap(kMapIDToEE).at(ee);
    mono::HyqlegInverseKinematics::KneeBend bend = mono::HyqlegInverseKinematics::Forward;

    if (foot == LF) {
      ee_pos_H = pos_B.At(ee);
    } else if (foot == RF) {
      ee_pos_H = pos_B.At(ee).cwiseProduct(Eigen::Vector3d(1,-1,1));
    } else if (foot == LH) {
      ee_pos_H = pos_B.At(ee).cwiseProduct(Eigen::Vector3d(-1,1,1));
      bend = mono::HyqlegInverseKinematics::Backward;
    } else if (foot == RH) {
      ee_pos_H = pos_B.At(ee).cwiseProduct(Eigen::Vector3d(-1,-1,1));
      bend = mono::HyqlegInverseKinematics::Backward;
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


} /* namespace quad */
} /* namespace xpp */


