#include <xpp_vis_hyq/inverse_kinematics_hyq4.h>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>

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
    switch (ee) {
      case LF:
        ee_pos_H = pos_B.at(ee);
        break;
      case RF:
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(1,-1,1));
        break;
      case LH:
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(-1,1,1));
        bend = HyqlegInverseKinematics::Backward;
        break;
      case RH:
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(-1,-1,1));
        bend = HyqlegInverseKinematics::Backward;
        break;
      default: assert(false); break; // foot id does not exist
        break;
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
