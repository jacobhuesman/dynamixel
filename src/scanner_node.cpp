#include <ros/ros.h>
#include <dynamixel.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

using namespace dynamixel;
using std_srvs::Empty;

bool start_scan;

bool scan_callback(Empty::Request &req, Empty::Response &res)
{
  start_scan = true;
  return true;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "zr300");
  ros::NodeHandle nh("~");
  ros::ServiceServer scan_service = nh.advertiseService("scan", scan_callback);
  tf2_ros::TransformBroadcaster broadcaster;
  ros::Publisher pub = nh.advertise<std_msgs::Bool>("mapping_is_good",1);
  ros::ServiceClient client_more = nh.serviceClient<std_srvs::SetBool>("/costmap_more_inflation/costmap/rock_layer/enable_mapping");
  ros::ServiceClient client_less = nh.serviceClient<std_srvs::SetBool>("/costmap_less_inflation/costmap/rock_layer/enable_mapping");

  std_srvs::SetBool en_map;
  std_msgs::Bool mapping_good;
  mapping_good.data = 0;
  start_scan = false;
  bool left_limit_hit = true, right_limit_hit = true;

  // Initialize Camera Objects
  if (mraa_get_platform_type() != MRAA_UP2)
  {
    throw cl_host_error("Host must be the UP2 board");
  }

  Dynamixel *servo = new Dynamixel(0x11, "zr300_dynamixel", "zr300_mount", 284, 739, 155, 0.5);

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
    if (start_scan)
    {
      left_limit_hit = false;
      right_limit_hit = false;
      start_scan = false;
      en_map.request.data = 1;
      client_more.call(en_map);
      client_less.call(en_map);
    }

    try
    {
      servo->updatePosition();
      ROS_DEBUG("Current Position: %i", servo->getCurrentPosition());
      if (!right_limit_hit || !left_limit_hit)
      {
        servo->scan();
        if (servo->getCurrentPosition() >= (739 - 10))
        {
          ROS_DEBUG("Hit left limit");
          left_limit_hit = true;
        }
        if (servo->getCurrentPosition() <= (284 + 10))
        {
          ROS_DEBUG("Hit right limit");
          right_limit_hit = true;
        }
      }
      else if (abs(servo->getCurrentPosition() - 512) > 10)
      {
        servo->setPosition(512);
        en_map.request.data = 0;
        client_more.call (en_map);
        client_less.call (en_map);
        mapping_good.data = 1;
      }
      broadcaster.sendTransform(servo->getTransformMsg());
    }
    catch (cl_error &e)
    {
      ROS_WARN("%s", e.what());
    }
    pub.publish(mapping_good);
    rate.sleep();
    ros::spinOnce();
  }
}