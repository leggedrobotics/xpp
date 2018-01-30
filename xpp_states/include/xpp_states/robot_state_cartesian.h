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

#ifndef _XPP_STATES_ROBOT_STATE_CARTESIAN_H_
#define _XPP_STATES_ROBOT_STATE_CARTESIAN_H_

#include <xpp_states/state.h>
#include <xpp_states/endeffectors.h>

namespace xpp {

/**
 * @brief Defines a complete robot state in Cartesian space.
 *
 * This is the alternative represenation to defining the robot by it's joint
 * angles. Here each endeffector (foot, hand) is given by a 3D-state and
 * joints angles could be obtained through Inverse Kinematics.
 *
 * see also robot_state_joint.h.
 */
class RobotStateCartesian {
public:

  /**
   * @brief  Constructs a zero initialized robot state with n_ee endeffectors.
   * @param  n_ee  Number of endeffectors.
   */
  RobotStateCartesian(int n_ee);
  ~RobotStateCartesian() = default;

  State3d base_;
  EndeffectorsMotion ee_motion_;
  Endeffectors<Vector3d> ee_forces_;
  EndeffectorsContact ee_contact_;
  double t_global_;
};

} /* namespace xpp */

#endif /* _XPP_STATES_ROBOT_STATE_CARTESIAN_H_ */
