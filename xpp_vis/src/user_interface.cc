
#include <xpp_vis/user_interface.h>

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <ros/node_handle.h>

#include <xpp_msgs/UserCommand.h>

#include <xpp_ros_conversions/convert.h>
#include <xpp_msgs/topic_names.h>

#include <xpp_states/terrain_types.h>


namespace xpp {

UserInterface::UserInterface ()
{
  ::ros::NodeHandle n;
  key_sub_ = n.subscribe("/keyboard/keydown", 1, &UserInterface::CallbackKeyboard, this);
  ROS_INFO("Subscribed to: %s", key_sub_.getTopic().c_str());

  user_command_pub_ = n.advertise<xpp_msgs::UserCommand>(xpp_msgs::user_command, 1);
  ROS_INFO("Publishing to: %s", user_command_pub_.getTopic().c_str());

  // publish goal zero initially
  goal_geom_.lin.p_.setZero();
  goal_geom_.lin.p_ << 1.3, 0.0, 0.46;
  goal_geom_.ang.p_ << 0.0, 0.0, 0.0; // roll, pitch, yaw angle applied Z->Y'->X''

  terrain_id_    = 0;
  gait_combo_id_ = 0;
}

void
UserInterface::CallbackKeyboard (const keyboard::Key& msg)
{
  const static double d_lin = 0.1;  // [m]
  const static double d_ang = 0.25; // [rad]

  switch (msg.code) {
    // desired goal positions
    case msg.KEY_RIGHT:
      goal_geom_.lin.p_.x() -= d_lin;
      ROS_INFO_STREAM("Goal position set to " << goal_geom_.lin.p_.transpose());
      break;
    case msg.KEY_LEFT:
      goal_geom_.lin.p_.x() += d_lin;
      ROS_INFO_STREAM("Goal position set to " << goal_geom_.lin.p_.transpose());
      break;
    case msg.KEY_DOWN:
      goal_geom_.lin.p_.y() += d_lin;
      ROS_INFO_STREAM("Goal position set to " << goal_geom_.lin.p_.transpose());
      break;
    case msg.KEY_UP:
      goal_geom_.lin.p_.y() -= d_lin;
      ROS_INFO_STREAM("Goal position set to " << goal_geom_.lin.p_.transpose());
      break;
    case msg.KEY_PAGEUP:
      goal_geom_.lin.p_.z() += 0.5*d_lin;
      ROS_INFO_STREAM("Goal position set to " << goal_geom_.lin.p_.transpose());
      break;
    case msg.KEY_PAGEDOWN:
      goal_geom_.lin.p_.z() -= 0.5*d_lin;
      ROS_INFO_STREAM("Goal position set to " << goal_geom_.lin.p_.transpose());
      break;

    // desired goal orientations
    case msg.KEY_KP4:
      goal_geom_.ang.p_.x() -= d_ang; // roll-
      break;
    case msg.KEY_KP6:
      goal_geom_.ang.p_.x() += d_ang; // roll+
      break;
    case msg.KEY_KP8:
      goal_geom_.ang.p_.y() += d_ang; // pitch+
      break;
    case msg.KEY_KP2:
      goal_geom_.ang.p_.y() -= d_ang; // pitch-
      break;
    case msg.KEY_KP1:
      goal_geom_.ang.p_.z() += d_ang; // yaw+
      break;
    case msg.KEY_KP9:
      goal_geom_.ang.p_.z() -= d_ang; // yaw-
      break;

    // terrains
    case msg.KEY_1:
      terrain_id_ = AdvanceCircularBuffer(terrain_id_, K_TERRAIN_COUNT-1);
      ROS_INFO_STREAM("Switched terrain to " << terrain_id_);
      break;

    case msg.KEY_g:
      gait_combo_id_ = AdvanceCircularBuffer(gait_combo_id_, kMaxNumGaits_);
      ROS_INFO_STREAM("Switched gait to combo " + std::to_string(gait_combo_id_) );
      break;

    // speed
    case msg.KEY_KP_PLUS:
      total_duration_ += 0.2;
      ROS_INFO_STREAM("Total duration increased to " << total_duration_);
    break;
    case msg.KEY_KP_MINUS:
      total_duration_ -= 0.2;
      ROS_INFO_STREAM("Total duration decreased to " << total_duration_);
    break;


    case msg.KEY_o:
      ROS_INFO_STREAM("Optimize motion request sent");
      optimize_ = true;
      break;
    case msg.KEY_p:
      ROS_INFO_STREAM("ATTENTION: Are you sure you want to send this to the robot?");
      ROS_INFO_STREAM("Press y and Enter in this window to continue...");
      char input;
      std::cin >> input;
      if (input == 'y') {
        publish_optimized_trajectory_ = true;
        ROS_INFO_STREAM("Publish optimized trajectory request sent");
      } else
        ROS_INFO_STREAM("Aborted");
      break;
    case msg.KEY_s:
      ROS_INFO_STREAM("Toggled NLP solver type");
      use_solver_snopt_ = !use_solver_snopt_;
      break;
    case msg.KEY_r:
      ROS_INFO_STREAM("Replaying already optimized trajectory");
      replay_trajectory_ = true;
      break;
    default:
      break;
  }

  PublishCommand();
}

void UserInterface::PublishCommand()
{
  xpp_msgs::UserCommand msg;
  msg.goal_lin          = Convert::ToRos(goal_geom_.lin);
  msg.goal_ang          = Convert::ToRos(goal_geom_.ang);
  msg.replay_trajectory = replay_trajectory_;
  msg.use_solver_snopt  = use_solver_snopt_;
  msg.optimize          = optimize_;
  msg.terrain_id        = terrain_id_;
  msg.gait_id           = gait_combo_id_;
  msg.total_duration    = total_duration_;
  msg.publish_traj      = publish_optimized_trajectory_;

  user_command_pub_.publish(msg);

  optimize_ = false;
  replay_trajectory_  = false;
  publish_optimized_trajectory_ = false;
}

int UserInterface::AdvanceCircularBuffer(int& curr, int max) const
{
  return curr==max? 0 : curr+1;
}

} /* namespace xpp */
