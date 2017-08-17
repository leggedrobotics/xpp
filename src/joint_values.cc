/**
 @file    joint_values.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

#include <xpp/joint_values.h>

namespace xpp {

using EEID = EndeffectorID;

JointValues::JointValues (int n_ee, int n_joints_per_leg, double value)
    : Base(n_ee)
{
  n_joints_per_leg_ = n_joints_per_leg;
  n_joints_ = n_ee * n_joints_per_leg;

  SetAll(VectorXd::Constant(n_joints_per_leg, value));
}

JointValues::~JointValues ()
{
}

int
JointValues::GetNumJoints () const
{
  return n_joints_;
}

JointValues::VectorXd
JointValues::ToVec (const EEOrder& ee_order) const
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
JointValues::SetFromVec (const VectorXd& xpp, const EEOrder& ee_order)
{
  int j = 0;

  for (EEID ee : ee_order) {
    At(ee) = xpp.middleRows(j, n_joints_per_leg_);
    j += n_joints_per_leg_;
  }
}

JointValues::VectorXd
JointValues::ToVec () const
{
  return ToVec(GetEEsOrdered());
}

void
JointValues::SetFromVec (const VectorXd& q)
{
  SetFromVec(q, GetEEsOrdered());
}

double&
JointValues::At (JointID joint)
{
  div_t result = std::div(joint, n_joints_per_leg_);
  EEID ee = static_cast<EEID>(result.quot);
  return At(ee)[result.rem];
}

double
JointValues::At (JointID joint) const
{
  return ToVec()[joint];
}

const JointValues
JointValues::operator + (const JointValues& rhs) const
{
  VectorXd result = ToVec() + rhs.ToVec();
  JointValues xpp(GetCount(), n_joints_per_leg_);
  xpp.SetFromVec(result);
  return xpp;
}

const JointValues
JointValues::operator * (double scalar) const
{
  VectorXd result = scalar*ToVec();
  JointValues xpp(GetCount(), n_joints_per_leg_);
  xpp.SetFromVec(result);
  return xpp;
}

int
JointValues::GetNumJointsPerEE () const
{
  return n_joints_per_leg_;
}

} /* namespace xpp */

