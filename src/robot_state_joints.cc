/**
@file    articulated_robot_state.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Captures the full state of the robot (body, feet)
 */

#include <xpp/robot_state_joints.h>

namespace xpp {

using Vector3d = Eigen::Vector3d;

RobotStateJoints::RobotStateJoints(int n_ee, int n_joints_per_ee)
    :RobotDataHolder(n_ee),
     q(n_ee, n_joints_per_ee),
     qd(n_ee, n_joints_per_ee),
     qdd(n_ee, n_joints_per_ee)
{
}

RobotStateJoints::~RobotStateJoints()
{
}

RobotStateJoints
RobotStateJoints::FromCartesian(const StateC& state, const InvKinPtr& ik)
{
  RobotStateJoints rob(state.GetEECount(), ik->GetJointsPerEE());

  rob.SetCommon(state.GetCommon());
  rob.SetJointAngles(state.GetEEPos(), ik);

  return rob;
}

//RobotStateJoints::StateC
//RobotStateJoints::ConvertToCartesian(const ForKinPtr& fk) const
//{
//  StateC cartesian(GetEECount());
//  cartesian.SetCommon(GetCommon());
//
//  auto W_p_ee = GetEEInWorld(fk);
//  auto W_v_ee = GetEEInVelWorld(fk);
//
//  cartesian.SetEEStateInWorld(kPos, W_p_ee);
//  cartesian.SetEEStateInWorld(kVel, W_v_ee);
//  // missing acceleration
//
//  return cartesian;
//}

void
RobotStateJoints::SetJointAngles (const PosEE& ee_W, const InvKinPtr& ik)
{
  // transform world -> base frame
  Eigen::Matrix3d B_R_W = GetBase().ang.q.normalized().toRotationMatrix().inverse();

  PosEE ee_B(GetEECount());
  for (EEID ee : GetEndeffectors())
    ee_B.At(ee) = B_R_W * (ee_W.At(ee) - GetBase().lin.p_);

  q = ik->GetAllJointAngles(ee_B);
}

//RobotStateJoints::PosEE
//RobotStateJoints::GetEEInWorld (const ForKinPtr& fk) const
//{
//  Eigen::Matrix3d W_R_B = GetBase().ang.q.normalized().toRotationMatrix();
//
//  fk->SetCurrent(q, qd, W_R_B);
//  auto ee_B = fk->getEEPosInBase();
//
//  PosEE ee_W(GetEECount());
//  for (EEID ee : GetEndeffectors())
//    ee_W.At(ee) = W_R_B * ee_B.At(ee) + GetBase().lin.p_;
//
//  return ee_W;
//}

//RobotStateJoints::VelEE
//RobotStateJoints::GetEEInVelWorld (const ForKinPtr& fk) const
//{
//  Eigen::Matrix3d W_R_B = GetBase().ang.q.normalized().toRotationMatrix();
//  fk->SetCurrent(q, qd, W_R_B);
//
//  auto B_v_joints = fk->GetEEVelInBase();
//  auto B_p_ee     = fk->getEEPosInBase();
//
//  VelEE W_v_ee(GetEECount());
//  for (EEID ee : GetEndeffectors()) {
//    Vector3d B_v_base_ang = GetBase().ang.v.cross(B_p_ee.At(ee));
//    Vector3d vel_W = GetBase().lin.v_ + W_R_B * (B_v_joints.At(ee) + B_v_base_ang);
//    W_v_ee.At(ee) = vel_W;
//  }
//
//  return W_v_ee;
//}

RobotStateJoints::StateJVec
RobotStateJoints::BuildWholeBodyTrajectory (const StateCVec& trajectory_in,
                                            const InvKinPtr& ik)
{
  StateJVec trajectory_joints;

  Eigen::VectorXd q_prev, qd_prev;

  double t_prev = 0.0;
  bool first_state = true;
  for (const auto& state : trajectory_in) {

    RobotStateJoints rob = FromCartesian(state, ik);

    // joint velocity
    if (!first_state) { // to avoid jump in vel/acc in first state
      double dt = state.GetTime() - t_prev;
      rob.qd.SetFromVec(  (rob.q.ToVec()  - q_prev)  / dt);
      rob.qdd.SetFromVec( (rob.qd.ToVec() - qd_prev) / dt);
    }

    trajectory_joints.push_back(rob);
    q_prev  = rob.q.ToVec();
    qd_prev = rob.qd.ToVec();
    t_prev  = state.GetTime();
    first_state = false;
  }

  return trajectory_joints;
}

} // namespace xpp
