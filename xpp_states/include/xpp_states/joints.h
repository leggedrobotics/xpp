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

#ifndef _XPP_STATES_JOINTS_H_
#define _XPP_STATES_JOINTS_H_

#include <vector>
#include <Eigen/Dense>

#include <xpp_states/endeffectors.h>

namespace xpp {

/**
 * @brief Container to access joint values of each endeffectors.
 *
 * The idea is that every joint affects only one specific endeffector, so the
 * joints are grouped in this fashion. They can also be transformed to or set
 * from an undiscriminative Eigen::VectorXd. This however is not recommended,
 * as this Cartesian <->joint relationship is then lost/hidden.
 */
class Joints : public Endeffectors<VectorXd> {
public:
  using Base = Endeffectors<VectorXd>;
  using Base::GetEECount;
  using Base::at;
  using EEOrder = std::vector<EndeffectorID>;
  using JointID = uint;

  /**
   * @brief  Constructs joint values all set to value.
   * @param  n_ee             total number of endeffectors.
   * @param  n_joints_per_ee  number of joints for each endeffector.
   * @param  value            same joint value set for each joint.
   *
   * Constructing joints just from the number of joints is not permitted,
   * as we enforce every joint to be assigned to an endeffector. This is
   * restrictive, however it helps to avoid bugs and makes later computations
   * easier.
   */
  explicit Joints (int n_ee, int n_joints_per_ee, double value = 0.0);

  /**
   * @brief Converts a vector of leg joints into a Joint representation.
   * @param joints  Joint values of each endeffector
   *
   * Attention: Each endeffector must have the same number of joints.
   */
  explicit Joints (const std::vector<VectorXd>& joints);
  virtual ~Joints () = default;

  /**
   * @brief Converts joint values to Eigen vector.
   *
   * The endeffectors are starting from zero and for each endeffector
   * the joints are appended in the order they where inserted.
   */
  VectorXd ToVec() const;

  /**
   * @brief Converts joint values to Eigen vector according to specific order.
   * @param ee_order  The order in which the endeffector's joints are appended.
   */
  VectorXd ToVec(const EEOrder& ee_order) const;

  /**
   * @brief Sets joints values from Eigen vector.
   * @param q The Eigen Vector of joint values.
   *
   * The vector q is interpreted as if the top n_joints_per_leg values
   * rows belong to endeffector 0, the next n_joints_per_leg values to
   * endeffector 1 and so on.
   */
  void SetFromVec(const VectorXd& q);

  /**
   * @brief Sets joint values from Eigen vector in specific order.
   * @param q         The Eigen Vector of joint values.
   * @param ee_order  Describes the internal order of q.
   */
  void SetFromVec(const VectorXd& q, const EEOrder& ee_order);

  /**
   * @returns read/write access of the joint value at index joint.
   */
  double& GetJoint(JointID joint);

  /**
   * @returns read access of the joint value at index joint.
   */
  double GetJoint(JointID joint) const;

  int GetNumJoints() const;
  int GetNumJointsPerEE() const;

private:
  int n_joints_per_leg_;
  int n_joints_;
};

} /* namespace xpp */

#endif /* _XPP_STATES_JOINTS_H_ */
