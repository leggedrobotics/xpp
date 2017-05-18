/**
 @file    hyq_rviz_visualizer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 24, 2016
 */

#ifndef HYQ_RVIZ_VISUALIZER_H_
#define HYQ_RVIZ_VISUALIZER_H_

#include <xpp/robot_rviz_visualizer.h>

namespace xpp {

/**
 * @brief Transforms a Cartesian state to HyQ joint state.
 */
class HyqRvizVisualizer : public RobotRvizVisualizer {
public:
	HyqRvizVisualizer() {};
	virtual ~HyqRvizVisualizer() {};

	void SetJointsFromRos(const sensor_msgs::JointState &msg,
	                               NameJointAngleMap& q);

	virtual void StateCallback(const StateMsg& msg);
};

} // namespace xpp

#endif /* HYQ_RVIZ_VISUALIZER_H_ */
