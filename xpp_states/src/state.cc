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

#include <vector>

#include <xpp_states/state.h>

namespace xpp {

StateLinXd::StateLinXd (int dim)
{
  kNumDim = dim;
  p_ = VectorXd::Zero(dim);
  v_ = VectorXd::Zero(dim);
  a_ = VectorXd::Zero(dim);
}

StateLinXd::StateLinXd (const VectorXd& _p,
                        const VectorXd& _v,
                        const VectorXd& _a)
    :StateLinXd(_p.rows())
{
  p_ = _p;
  v_ = _v;
  a_ = _a;
}

StateLinXd::StateLinXd (const VectorXd& _p)
    :StateLinXd(_p.rows())
{
  p_ = _p;
}

const VectorXd
StateLinXd::GetByIndex (MotionDerivative deriv) const
{
  switch (deriv) {
    case kPos:  return p_; break;
    case kVel:  return v_; break;
    case kAcc:  return a_; break;
    default: throw std::runtime_error("[StateLinXd::GetByIndex] derivative not part of state");
  }
}

VectorXd&
StateLinXd::GetByIndex (MotionDerivative deriv)
{
  switch (deriv) {
    case kPos:  return p_; break;
    case kVel:  return v_; break;
    case kAcc:  return a_; break;
    default: throw std::runtime_error("[StateLinXd::GetByIndex] derivative not part of state");
  }
}

StateLin3d::StateLin3d (const StateLinXd& state_xd) : StateLinXd(3)
{
  assert(state_xd.kNumDim == 3);

  p_ = state_xd.p_;
  v_ = state_xd.v_;
  a_ = state_xd.a_;
}

StateLin2d
StateLin3d::Get2D() const
{
  StateLin2d p2d;
  p2d.p_ = p_.topRows<kDim2d>();
  p2d.v_ = v_.topRows<kDim2d>();
  p2d.a_ = a_.topRows<kDim2d>();
  return p2d;
}

Vector6d
State3d::Get6dVel () const
{
  Vector6d h_xd;
  h_xd.segment(AX, 3) = ang.w;
  h_xd.segment(LX, 3) = lin.v_;
  return h_xd;
}

Vector6d
State3d::Get6dAcc () const
{
  Vector6d h_xdd;
  h_xdd.segment(AX, 3) =  ang.wd;
  h_xdd.segment(LX, 3) =  lin.a_;
  return h_xdd;
}

} // namespace xpp

