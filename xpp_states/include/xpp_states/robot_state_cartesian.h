
#ifndef _XPP_STATES_ROBOT_STATE_CARTESIAN_H_
#define _XPP_STATES_ROBOT_STATE_CARTESIAN_H_

#include <xpp_states/state.h>
#include <xpp_states/endeffectors.h>

namespace xpp {

/**
 * @brief Defines a complete robot state in Cartesian space.
 *
 * This is the alternative represenation to defining the robot by it's joint
 * angles. Here each endeffector (foot, hand) is given by a 3D-state and
 * joints angles could be obtained through Inverse Kinematics.
 *
 * see also robot_state_joint.h.
 */
class RobotStateCartesian {
public:

  /**
   * @brief  Constructs a zero initialized robot state with n_ee endeffectors.
   * @param  n_ee  Number of endeffectors.
   */
  RobotStateCartesian(int n_ee);
  ~RobotStateCartesian() {};

  State3d base_;
  EndeffectorsMotion ee_motion_;
  Endeffectors<Vector3d> ee_forces_;
  EndeffectorsContact ee_contact_;
  double t_global_;
};

} /* namespace xpp */

#endif /* _XPP_STATES_ROBOT_STATE_CARTESIAN_H_ */
