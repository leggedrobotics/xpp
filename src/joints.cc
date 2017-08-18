/**
 @file    joint_values.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

#include <../include/xpp/joints.h>

namespace xpp {

using EEID = EndeffectorID;

Joints::Joints (int n_ee, int n_joints_per_leg, double value)
    : Base(n_ee)
{
  n_joints_per_leg_ = n_joints_per_leg;
  n_joints_ = n_ee * n_joints_per_leg;

  SetAll(VectorXd::Constant(n_joints_per_leg, value));
}

Joints::Joints (const std::vector<VectorXd>& q_vec)
    : Base(q_vec.size())
{
  n_joints_per_leg_ = q_vec.front().rows(); // assume all are the same
  n_joints_ = GetCount()*n_joints_per_leg_;

  for (auto ee : GetEEsOrdered())
    At(ee) = q_vec.at(ee);
}

Joints::~Joints ()
{
}

int
Joints::GetNumJoints () const
{
  return n_joints_;
}

Joints::VectorXd
Joints::ToVec (const EEOrder& ee_order) const
{
  VectorXd q_combined(n_joints_);
  int j = 0;

  for (EEID ee : ee_order) {
    q_combined.middleRows(j, n_joints_per_leg_) = At(ee);
    j += n_joints_per_leg_;
  }

  return q_combined;
}

void
Joints::SetFromVec (const VectorXd& xpp, const EEOrder& ee_order)
{
  int j = 0;

  for (EEID ee : ee_order) {
    At(ee) = xpp.middleRows(j, n_joints_per_leg_);
    j += n_joints_per_leg_;
  }
}

Joints::VectorXd
Joints::ToVec () const
{
  return ToVec(GetEEsOrdered());
}

void
Joints::SetFromVec (const VectorXd& q)
{
  SetFromVec(q, GetEEsOrdered());
}

double&
Joints::At (JointID joint)
{
  div_t result = std::div(joint, n_joints_per_leg_);
  EEID ee = static_cast<EEID>(result.quot);
  return At(ee)[result.rem];
}

double
Joints::At (JointID joint) const
{
  return ToVec()[joint];
}

const Joints
Joints::operator + (const Joints& rhs) const
{
  VectorXd result = ToVec() + rhs.ToVec();
  Joints xpp(GetCount(), n_joints_per_leg_);
  xpp.SetFromVec(result);
  return xpp;
}

const Joints
Joints::operator * (double scalar) const
{
  VectorXd result = scalar*ToVec();
  Joints xpp(GetCount(), n_joints_per_leg_);
  xpp.SetFromVec(result);
  return xpp;
}

int
Joints::GetNumJointsPerEE () const
{
  return n_joints_per_leg_;
}

} /* namespace xpp */

