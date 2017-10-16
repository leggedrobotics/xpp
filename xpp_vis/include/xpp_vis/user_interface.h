
#ifndef XPP_VIS_USER_INTERFACE_H_
#define XPP_VIS_USER_INTERFACE_H_

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Vector3.h>
#include <keyboard/Key.h>

#include <xpp_states/state.h>


namespace xpp {

/**
 * @brief Translates user input into a ROS message.
 *
 * This includes high level input about where to go (e.g. converting
 * keyboard input into a goal state), which terrain to visualize, etc.
 *
 * See the CallbackKeyboard function for the Key->Action mappings.
 */
class UserInterface {
public:

  /**
   * @brief  Constructs default object to interact with framework.
   */
  UserInterface ();
  virtual ~UserInterface () {};

private:
  ::ros::Subscriber key_sub_;          ///< the input key hits to the node.
  ::ros::Publisher  user_command_pub_; ///< the output message of the node.

  void CallbackKeyboard(const keyboard::Key& msg);
  void PublishCommand();

  State3dEuler goal_geom_;
  int kMaxNumGaits_ = 8;
  int terrain_id_;
  int gait_combo_id_;
  bool replay_trajectory_ = false;
  bool use_solver_snopt_ = false;
  bool optimize_ = false;
  bool publish_optimized_trajectory_ = false;
  double total_duration_ = 2.0;

  int AdvanceCircularBuffer(int& curr, int max) const;
};

} /* namespace xpp */

#endif /* XPP_VIS_USER_INTERFACE_H_ */
