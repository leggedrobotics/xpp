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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <xpp_msgs/UserCommand.h>      // listen to goal state
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <xpp_states/convert.h>
#include <xpp_vis/rviz_robot_builder.h>
#include <xpp_vis/rviz_terrain_builder.h>


static ros::Publisher rviz_marker_pub;
static ros::Publisher rviz_pose_pub;
static xpp::RvizRobotBuilder robot_builder;
static xpp::RvizTerrainBuilder terrain_builder;

static void StateCallback (const xpp_msgs::RobotStateCartesian& state_msg)
{
  auto rviz_marker_msg = robot_builder.BuildRobotState(state_msg);
  rviz_marker_pub.publish(rviz_marker_msg);
}

static void TerrainInfoCallback (const xpp_msgs::TerrainInfo& terrain_msg)
{
  robot_builder.SetTerrainParameters(terrain_msg);
}

static void ParamsCallback (const xpp_msgs::OptParameters& params_msg)
{
  ROS_DEBUG_STREAM("received current optimization parameters");
  robot_builder.SetOptimizationParameters(params_msg);
}

void CallbackUserCommand(const xpp_msgs::UserCommand& msg_in)
{
  // add some terrain
  auto msg = terrain_builder.BuildTerrain(msg_in.terrain_id);
  rviz_marker_pub.publish(msg);

  // publish goal pose
  auto goal_msg = terrain_builder.BuildPose(msg_in.goal_lin.pos,
                                                msg_in.goal_ang);
  rviz_pose_pub.publish(goal_msg);
}

int main(int argc, char *argv[])
{
  using namespace ros;

	init(argc, argv, "rviz_marker_visualizer");

	NodeHandle n;

  Subscriber parameters_sub;
  parameters_sub = n.subscribe(xpp_msgs::opt_parameters, 1, ParamsCallback);

	Subscriber state_sub_curr, state_sub_des, terrain_info_sub;
	state_sub_curr    = n.subscribe(xpp_msgs::robot_state_current, 1, StateCallback);
	state_sub_des     = n.subscribe(xpp_msgs::robot_state_desired, 1, StateCallback);
	terrain_info_sub  = n.subscribe(xpp_msgs::terrain_info, 1,  TerrainInfoCallback);

	Subscriber goal_sub;
	goal_sub = n.subscribe(xpp_msgs::user_command, 1, CallbackUserCommand);

	rviz_marker_pub = n.advertise<visualization_msgs::MarkerArray>("xpp/rviz_markers", 1);
	rviz_pose_pub   = n.advertise<geometry_msgs::PoseStamped>("xpp/goal", 1);

	spin();

	return 1;
}
