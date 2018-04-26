#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <boost/program_options.hpp>

#include <dynamixel.h>
#include <comm_layer_defs.h>

using namespace dynamixel;
using namespace boost::program_options;

int main(int argc, char *argv[])
{
  ros::Time::init();
  Dynamixel servo(0x11);

  try
  {
    options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("setPosition,s",  value<uint16_t>(), "Set Position")
        ("getPosition,g", "Get Position")
        ("setVelocity,v",  value<uint16_t>(), "Set Velocity")
        ("adjustCamera,a", value<int16_t>(), "Adjust Camera Velocity")
        ("setPollingDt,t", value<uint16_t>(), "Set Polling Dt")
        ("testPolling",    value<std::vector<uint16_t>>()->multitoken(),  "Test given polling time");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help"))
    {
      std::cout << desc << '\n';
    }
    if (vm.count("setPosition"))
    {
      uint16_t position = vm["setPosition"].as<uint16_t>();
      try
      {
        std::cout << "Setting position: " << position << "... ";
        servo.setPosition(position);
        std::cout << "Success" << std::endl;
      }
      catch (cl_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("getPosition"))
    {
      uint16_t position;
      try
      {
        std::cout << "Reading position... ";
        servo.getPosition(&position);
        std::cout << position << " Success" << std::endl;
      }
      catch (cl_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("setVelocity"))
    {
      uint16_t velocity = vm["setVelocity"].as<uint16_t>();
      try
      {
        std::cout << "Setting velocity: " << velocity << "...";
        servo.setVelocity(velocity);
        std::cout << "Success" << std::endl;
      }
      catch (cl_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("adjustCamera"))
    {
      int16_t velocity = vm["adjustCamera"].as<int16_t>();
      try
      {
        std::cout << "Adjusting camera velocity to: " << velocity << "... ";
        servo.adjustCamera(velocity);
        std::cout << "Success" << std::endl;
      }
      catch (cl_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("setPollingDt"))
    {
      uint16_t polling_dt = vm["setPollingDt"].as<uint16_t>();
      try
      {
        std::cout << "Setting polling dt: " << polling_dt << "... ";
        servo.setPollingDt(polling_dt);
        std::cout << "Success" << std::endl;
      }
      catch (cl_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("testPolling"))
    {
      // TODO rework and add try catch blocks
      std::vector<uint16_t> options = vm["testPolling"].as<std::vector<uint16_t>>();
      uint16_t polling_dt = options[0];
      uint16_t polling_velocity = options[1];
      std::cout << "Setting polling dt to: " << polling_dt << " Setting velocity to: " << polling_velocity << std::endl;

      std::cout << "Setting polling rate to 200 ms" << std::endl;
      servo.setPollingDt(200); // Set to known good polling rate first
      usleep(100000);
      std::cout << "Setting velocity to 100" << std::endl;
      servo.setVelocity(100);
      usleep(100000);
      std::cout << "Setting position to 0" << std::endl;
      servo.setPosition(0);
      uint16_t position;
      servo.getPosition(&position);
      while (position > 20)
      {
        sleep(1);
        servo.getPosition(&position);
      }
      usleep(100000);
      servo.setPollingDt(polling_dt);
      usleep(100000);
      servo.setVelocity(polling_velocity);
      usleep(100000);
      servo.setPosition(1023);
      usleep(100000);
      servo.getPosition(&position);
      int count = 0;
      int direction = 0;
      while (position < 1013)
      {
        usleep(30000);
        servo.getPosition(&position);
        if (count++ > 2)
        {
          count = 0;
          if (direction == 1)
          {
            polling_velocity++;
          }
          else
          {
            polling_velocity--;
          }
          if (polling_velocity > 80)
          {
            direction = 0;
          }
          if (polling_velocity < 50)
          {
            direction = 1;
          }
          servo.setVelocity(polling_velocity);
        }
        std::cout << "Current position: " << position << ", Current Velocity: " << polling_velocity << std::endl;
      }
      usleep(100000);
      servo.setPosition(0);
      usleep(100000);
      servo.getPosition(&position);
      while (position > 12)
      {
        usleep(30000);
        servo.getPosition(&position);
        if (count++ > 2)
        {
          count = 0;
          if (direction == 1)
          {
            polling_velocity++;
          }
          else
          {
            polling_velocity--;
          }
          if (polling_velocity > 80)
          {
            direction = 0;
          }
          if (polling_velocity < 50)
          {
            direction = 1;
          }
          servo.setVelocity(polling_velocity);
        }
        std::cout << "Current position: " << position << ", Current Velocity: " << polling_velocity << std::endl;
      }
      std::cout << "Finished" << std::endl;
    }
  }
  catch (const error &ex)
  {
    std::cerr << ex.what() << '\n';
  }
}
