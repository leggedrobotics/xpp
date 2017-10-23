/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <rosbag/bag.h>

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_ros_conversions/convert.h>
#include <xpp_states/robot_state_cartesian.h>


using namespace xpp;

int main(int argc, char *argv[])
{
  // creates a bag file wherever this executable is run from
  rosbag::Bag bag;
  bag.open("mono_traj.bag", rosbag::bagmode::Write);

  // visualize the state of a one-legged hopper
  RobotStateCartesian hopper(1);

  // create a sequence of states for a total duration of T spaced 0.01s apart.
  double T = 2.0;
  double dt = 0.01;
  double t = 1e-6;
  while (t < T)
  {
    // base and foot follow half a sine motion up and down
    hopper.base_.lin.p_.z() = 0.7 - 0.05*sin(2*M_PI/(2*T)*t);  //[m]
    hopper.ee_motion_.at(0).p_.z() = 0.1*sin(2*M_PI/(2*T)*t); // [m]
    hopper.ee_forces_.at(0).z() = 100; // [N]
    hopper.ee_contact_.at(0) = true;

    // save the state message with current time in the bag
    auto timestamp = ::ros::Time(t);
    auto msg = Convert::ToRos(hopper);
    bag.write(xpp_msgs::robot_state_desired, timestamp, msg);

    t += dt;
  }

  std::string bag_file = bag.getFileName(); // save location before closing
  bag.close();

  // plays back the states at desired speeds.
  // can now also use all the tools from rosbag to pause, speed-up, inspect,
  // debug the trajectory.
  // e.g. rosbag -play -r 0.2 /path/to/bag.bag
  system(("rosbag play " + bag_file).c_str());

  return 0;
}

