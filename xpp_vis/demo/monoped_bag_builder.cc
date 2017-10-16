/**
@file    monoped_bag_builder.cpp
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    4.03.2016
@brief   Start the nlp foothold optimization server
 */


#include <rosbag/bag.h>

#include <xpp_msgs/RobotStateCartesian.h>

#include <../../xpp_ros_conversions/include/xpp_ros_conversions/convert.h>
#include <xpp_msgs/topic_names.h>

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
    hopper.ee_motion_.At(E0).p_.z() = 0.1*sin(2*M_PI/(2*T)*t); // [m]
    hopper.ee_forces_.At(E0).z() = 100; // [N]
    hopper.ee_contact_.At(E0) = true;

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

  return 1;
}

