
#include <ros/init.h>

#include <xpp_vis/user_interface.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "user_iterface_node");

  xpp::UserInterface keyboard_user_interface;

  ros::spin();

  return 1;
}

