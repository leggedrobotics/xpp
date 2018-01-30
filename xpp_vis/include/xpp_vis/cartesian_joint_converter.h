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

#ifndef XPP_VIS_CARTESIAN_JOINT_CONVERTER_H_
#define XPP_VIS_CARTESIAN_JOINT_CONVERTER_H_

#include <string>

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <xpp_msgs/RobotStateCartesian.h>

#include "inverse_kinematics.h"

namespace xpp {

/**
 * @brief Converts a Cartesian robot representation to joint angles.
 *
 * This class subscribes to a Cartesian robot state message and publishes
 * the, through inverse kinematics converted, joint state message. This
 * can then be used to visualize URDFs in RVIZ.
 */
class CartesianJointConverter {
public:
  /**
   * @brief Creates a converter initializing the subscriber and publisher.
   * @param  ik  The %InverseKinematics to use for conversion.
   * @param  cart_topic  The ROS topic containing the Cartesian robot state.
   * @param  joint_topic The ROS topic to publish for the URDF visualization.
   */
  CartesianJointConverter (const InverseKinematics::Ptr& ik,
                           const std::string& cart_topic,
                           const std::string& joint_topic);
  virtual ~CartesianJointConverter () = default;

private:
  void StateCallback(const xpp_msgs::RobotStateCartesian& msg);

  ros::Subscriber cart_state_sub_;
  ros::Publisher  joint_state_pub_;

  InverseKinematics::Ptr inverse_kinematics_;
};

} /* namespace xpp */

#endif /* XPP_VIS_CARTESIAN_JOINT_CONVERTER_H_ */
