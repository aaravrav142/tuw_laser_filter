#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_line_sm");

  nodelet::Loader manager(false);
  nodelet::M_string remappings;
  nodelet::V_string my_argv(argv + 1, argv + argc);

  manager.load(ros::this_node::getName(), "tuw_laser_line_detection/tuw_laser_line_sm", remappings, my_argv);

  ros::spin();
  return 0;
}