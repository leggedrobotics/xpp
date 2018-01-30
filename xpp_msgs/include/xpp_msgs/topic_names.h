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

/**
 * @file topic_names.h
 *
 * Defines ROS topic names to be used to more reliably connect publisher and
 * subscriber nodes.
 */

#ifndef XPP_MSGS_TOPIC_NAMES_H_
#define XPP_MSGS_TOPIC_NAMES_H_

#include <string>

namespace xpp_msgs {

// the current robot cartesian state including base, feet, time, ...
static const std::string robot_state_current("/xpp/state_curr");

// the desired state that comes from the optimizer
static const std::string robot_state_desired("/xpp/state_des");

// desired joint state (equivalent to desired cartesian state
static const std::string joint_desired("/xpp/joint_des");

// sequence of desired states coming from the optimizer
static const std::string robot_trajectory_desired("/xpp/trajectory_des");

// parameters describing the robot kinematics
static const std::string robot_parameters("/xpp/params");

// information about terrain normals and friction coefficients
static const std::string terrain_info("/xpp/terrain_info");
}

#endif /* XPP_MSGS_TOPIC_NAMES_H_ */
