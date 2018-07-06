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

#include <xpp_hyq/inverse_kinematics_hyq4.h>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>

namespace xpp {

Joints
InverseKinematicsHyq4::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  Vector3d ee_pos_H; // foothold expressed in hip frame
  std::vector<Eigen::VectorXd> q_vec;

  // make sure always exactly 4 elements
  auto pos_B = x_B.ToImpl();
  pos_B.resize(4, pos_B.front());

  for (int ee=0; ee<pos_B.size(); ++ee) {

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
      default: // joint angles for this foot do not exist
        break;
    }

    ee_pos_H -= base2hip_LF_;
    q_vec.push_back(leg.GetJointAngles(ee_pos_H, bend));
  }

  return Joints(q_vec);
}

} /* namespace xpp */
