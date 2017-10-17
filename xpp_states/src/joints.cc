
#include <xpp_states/joints.h>

namespace xpp {


Joints::Joints (int n_ee, int n_joints_per_leg, double value)
    : Base(n_ee)
{
  n_joints_per_leg_ = n_joints_per_leg;
  n_joints_ = n_ee * n_joints_per_leg;

  SetAll(VectorXd::Constant(n_joints_per_leg, value));
}

Joints::Joints (const std::vector<VectorXd>& q_vec)
    : Joints(q_vec.size(), q_vec.front().rows())
{
  for (auto ee : GetEEsOrdered())
    at(ee) = q_vec.at(ee);
}

int
Joints::GetNumJoints () const
{
  return n_joints_;
}

int
Joints::GetNumJointsPerEE () const
{
  return n_joints_per_leg_;
}

VectorXd
Joints::ToVec (const EEOrder& ee_order) const
{
  VectorXd q_combined(n_joints_);
  int j = 0;

  for (auto ee : ee_order) {
    q_combined.middleRows(j, n_joints_per_leg_) = at(ee);
    j += n_joints_per_leg_;
  }

  return q_combined;
}

void
Joints::SetFromVec (const VectorXd& xpp, const EEOrder& ee_order)
{
  int j = 0;

  for (auto ee : ee_order) {
    at(ee) = xpp.middleRows(j, n_joints_per_leg_);
    j += n_joints_per_leg_;
  }
}

VectorXd
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
Joints::GetJoint (JointID joint)
{
  div_t result = std::div(joint, n_joints_per_leg_);
  EndeffectorID ee = result.quot;
  return at(ee)[result.rem];
}

double
Joints::GetJoint (JointID joint) const
{
  return ToVec()[joint];
}

Joints::~Joints ()
{
}

} /* namespace xpp */
