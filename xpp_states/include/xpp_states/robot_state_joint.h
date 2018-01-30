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

#ifndef XPP_STATES_ROBOT_STATE_JOINT_H_
#define XPP_STATES_ROBOT_STATE_JOINT_H_

#include <xpp_states/state.h>
#include <xpp_states/endeffectors.h>
#include <xpp_states/joints.h>

namespace xpp {

/**
 * @brief Defines a complete robot state in joint space.
 *
 * This is the alternative representation to defining the robot by its
 * endeffector state.
 *
 * see also robot_state_joint.h.
 */
class RobotStateJoint {
public:
  /**
   * @brief  Constructs a zero initialized robot state.
   * @param  n_ee  Number of endeffectors (hands, feet).
   * @param  n_joints_per_ee  Number of joints for each endeffector.
   */
  RobotStateJoint (int n_ee, int n_joints_per_ee);
  virtual ~RobotStateJoint () = default;

  State3d base_;
  Joints q_, qd_, qdd_;
  Joints torques_;
  EndeffectorsContact ee_contact_;
  double t_global_;
};

} /* namespace xpp */

#endif /* XPP_STATES_ROBOT_STATE_JOINT_H_ */
