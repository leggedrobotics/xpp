#include <xpp/quad/hyq_inverse_kinematics.h>

#include <xpp/cartesian_declarations.h>
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

    FootID foot = kMapOptToQuad.at(ee);
    mono::HyqlegInverseKinematics::KneeBend bend = mono::HyqlegInverseKinematics::Forward;

    switch (foot) {
      case LF:
        ee_pos_H = pos_B.At(ee);
        break;
      case RF:
        ee_pos_H = pos_B.At(ee).cwiseProduct(Eigen::Vector3d(1,-1,1));
        break;
      case LH:
        ee_pos_H = pos_B.At(ee).cwiseProduct(Eigen::Vector3d(-1,1,1));
        bend = mono::HyqlegInverseKinematics::Backward;
        break;
      case RH:
        ee_pos_H = pos_B.At(ee).cwiseProduct(Eigen::Vector3d(-1,-1,1));
        bend = mono::HyqlegInverseKinematics::Backward;
        break;
      default: assert(false);
        break;
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


