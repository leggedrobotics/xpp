/**
@file    topic_names.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 5, 2016
@brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_MSGS_INCLUDE_XPP_MSGS_TOPIC_NAMES_H_
#define USER_TASK_DEPENDS_XPP_MSGS_INCLUDE_XPP_MSGS_TOPIC_NAMES_H_

namespace xpp_msgs {

// command that tells the walking controller to start executing
static const std::string start_walking_topic("xpp/start_walking");

// position of the desired goal to reach
static const std::string goal_state_topic("xpp/goal_state");

// information necessary to setup an NLP
static const std::string req_info_nlp("required_info_nlp");

// sequence of full body hyq states as reference for a walking controller
static const std::string robot_trajectory("robot_trajectory");

// rviz marker topic for NLP's optimized variables (spline coefficients, footholds,...)
static const std::string rviz_optimized("/optimization_variables");

// rviz marker topic for fixed variables of NLP optimization (e.g. start stance)
static const std::string rviz_fixed("optimization_fixed");

}


#endif /* USER_TASK_DEPENDS_XPP_MSGS_INCLUDE_XPP_MSGS_TOPIC_NAMES_H_ */
