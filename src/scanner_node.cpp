#include <ros/ros.h>
#include <dynamixel.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace dynamixel;
using std_srvs::SetBool;

bool scan;

bool scan_callback(SetBool::Request &req, SetBool::Response &res)
{
  scan = req.data;
  if (scan)
  {
    ROS_INFO("Set scan to true");
  }
  else
  {
    ROS_INFO("Set scan to false");
  }
  res.success = 1;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "zr300");
  ros::NodeHandle nh("~");
  ros::ServiceServer scan_service = nh.advertiseService("scan", scan_callback);
  tf2_ros::TransformBroadcaster broadcaster;

  scan = false;

  // Initialize Camera Objects
  if (mraa_get_platform_type() != MRAA_UP2)
  {
    throw cl_host_error("Host must be the UP2 board");
  }

  Dynamixel *servo = new Dynamixel(0x11, "zr300_dynamixel", "zr300_mount", 284, 739, 28, 0.5);

  try
  {
    servo->setVelocity(0.5 * 86.0297 / 2);
  }
  catch (cl_error &e)
  {
    ROS_WARN("%s", e.what());
  }

  ros::Rate rate(30);
  while(ros::ok())
  {
    try
    {
      servo->updatePosition();
      ROS_INFO("Current Position: %i", servo->getCurrentPosition());
      if (scan)
      {
        servo->scan();
      }
      else if ((abs(servo->getCurrentPosition() - 512) > 10))
      {
        servo->setPosition(512);
      }
      broadcaster.sendTransform(servo->getTransformMsg());
    }
    catch (cl_error &e)
    {
      ROS_WARN("%s", e.what());
    }
    rate.sleep();
    ros::spinOnce();
  }
}