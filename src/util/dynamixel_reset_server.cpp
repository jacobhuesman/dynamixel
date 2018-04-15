#include "ros/ros.h"
#include <std_srvs/Trigger.h>
#include <mraa/gpio.hpp>

using std_srvs::Trigger;

mraa::Gpio* reset_pin;

bool reset_callback(Trigger::Request &req, Trigger::Response &res)
{
  mraa::Result result;

  result = reset_pin->dir(mraa::DIR_OUT);
  if (result != mraa::Result::SUCCESS)
  {
    res.message = "Failure";
    res.success = 0;
    ROS_ERROR("Unable to set pin direction to output, result: %i", result);
    return false;
  }

  result = reset_pin->write(0);
  if (result != mraa::Result::SUCCESS)
  {
    res.message = "Failure";
    res.success = 0;
    ROS_ERROR("Unable set pin low, result: %i", result);
    return false;
  }

  ros::Duration(1).sleep();

  result = reset_pin->dir(mraa::DIR_IN);
  if (result != mraa::Result::SUCCESS)
  {
    res.message = "Failure";
    res.success = 0;
    ROS_ERROR("Unable to set pin direction back to input, result: %i", result);
    return false;
  }

  res.message = "Board should be reset";
  res.success = 1;
  return true;
}

int main(int argc, char **argv)
{
  if (mraa_get_platform_type() == MRAA_UNKNOWN_PLATFORM || mraa_get_platform_type() == MRAA_NULL_PLATFORM)
  {
    throw std::runtime_error("Host does not support mraa::gpio");
  }

  ros::init(argc, argv, "dynamixel_reset_server");
  ros::NodeHandle nh;
  mraa::Result result;

  reset_pin = new mraa::Gpio(11); // Physical(11), BCM(17)
  ros::Duration(1.0).sleep(); // Wait for system to initialize

  result = reset_pin->dir(mraa::DIR_IN);
  if (result != mraa::Result::SUCCESS)
  {
    ROS_ERROR("Unable to set pin direction, result: %i", result);
  }

  ros::ServiceServer reset_service = nh.advertiseService("dynamixel_reset", reset_callback);
  ros::spin();

  result = reset_pin->dir(mraa::DIR_IN);
  if (result != mraa::Result::SUCCESS)
  {
    ROS_ERROR("Unable to set pin direction, result: %i", result);
  }

  return 0;
}