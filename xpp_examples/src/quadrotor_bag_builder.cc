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

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_states/convert.h>
#include <xpp_states/state.h>


using namespace xpp;

int main(int argc, char *argv[])
{
  // creates a bag file wherever executable is run (usually ~/.ros/)
  rosbag::Bag bag;
  bag.open("quadrotor_traj.bag", rosbag::bagmode::Write);

  // visualize the state of a drone (only has a base)
  State3d base;

  // create a sequence of states for a total duration of T spaced 0.01s apart.
  double T = 2*M_PI;
  double dt = 0.01;
  double t = 1e-6;
  while (t < T)
  {
    // base and foot follow half a sine motion up and down
    base.lin.p_.z() = 0.7 - 0.5*sin(t);  //[m]
    base.lin.p_.x() = 1.0/T*t;           //[m]

    double roll = 30./180*M_PI*sin(t);
    base.ang.q = GetQuaternionFromEulerZYX(0.0, 0.0, roll);

    // save the state message with current time in the bag
    auto timestamp = ::ros::Time(t);

    xpp_msgs::RobotStateJoint msg;
    msg.base = Convert::ToRos(base);
    bag.write("xpp/joint_quadrotor_des", timestamp, msg);

    t += dt;
  }

  std::string bag_file = bag.getFileName(); // save location before closing
  bag.close();

  // plays back the states at desired speeds.
  // can now also use all the tools from rosbag to pause, speed-up, inspect,
  // debug the trajectory.
  // e.g. rosbag -play -r 0.2 /path/to/bag.bag
  int success = system(("rosbag play " + bag_file).c_str());

  return 0;
}

