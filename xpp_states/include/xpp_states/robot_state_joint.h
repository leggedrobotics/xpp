
#ifndef XPP_STATES_ROBOT_STATE_JOINT_H_
#define XPP_STATES_ROBOT_STATE_JOINT_H_

#include <xpp_states/state.h>
#include <xpp_states/endeffectors.h>
#include <xpp_states/joints.h>

namespace xpp {

/**
 * @brief Defines a complete robot state in joint space.
 *
 * This is the alternative representation to defining the robot by it's
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
  virtual ~RobotStateJoint ();

  State3d base_;
  Joints q_, qd_, qdd_;
  Joints torques_;
  EndeffectorsContact ee_contact_;
  double t_global_;
};

} /* namespace xpp */

#endif /* XPP_STATES_ROBOT_STATE_JOINT_H_ */
