/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

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

} /* namespace xpp */
