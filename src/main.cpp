# include "../include/wallFollowing.h"

int main(int argc,char **argv)
{
  ros::init(argc, argv, "stdr_line_following", ros::init_options::AnonymousName);
  reactive_robot_feup::WallFollowing obj(argc, argv);
  ros::spin();
  return 0;
}
