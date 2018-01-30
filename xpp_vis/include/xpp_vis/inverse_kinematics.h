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

#ifndef XPP_VIS_INVERSE_KINEMATICS_H_
#define XPP_VIS_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>
#include <memory>

#include <xpp_states/endeffectors.h>
#include <xpp_states/joints.h>

namespace xpp {

/**
 *  @brief  Converts Cartesian endeffector positions into joint angles.
 *
 *  This class is responsible for calculating the joint angles of a robot
 *  given an endeffector position (=inverse kinematics).
 *  Base class that every inverse dynamics class must conform with.
 */
class InverseKinematics {
public:
  using Ptr      = std::shared_ptr<InverseKinematics>;
  using Vector3d = Eigen::Vector3d;

  InverseKinematics () = default;
  virtual ~InverseKinematics () = default;

  /**
    * @brief  Calculates the joint angles to reach a position @ pos_b.
    * @param  pos_b  3D-position of the endeffector expressed in base frame.
    * @return Joints angles of the robot.
    */
  virtual Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const = 0;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  virtual int GetEECount() const = 0;
};

} /* namespace xpp */

#endif /* XPP_VIS_INVERSE_KINEMATICS_H_ */
