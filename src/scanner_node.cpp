#include <ros/ros.h>
#include <dynamixel.h>

using namespace dynamixel;

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "node0");
  ros::NodeHandle nh("~");

  std::string name = ros::this_node::getName();
  Dynamixel *servo = new Dynamixel(0x11, name + "_dynamixel", name + "_camera_mount");

  // Initialize Camera Objects
  if (mraa_get_platform_type() != MRAA_UP2)
  {
    throw cl_host_error("Host must be the UP2 board");
  }

}