#ifndef DYNAMIXEL_HOST_LAYER_H
#define DYNAMIXEL_HOST_LAYER_H

#include <cstdint>
#include <cmath>

#include <mraa.hpp>

#include <comm_layer_defs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <dynamixel/DynamicServoConfig.h>


namespace dynamixel
{
  class I2cInterface
  {
  public:
    virtual ~I2cInterface() {};
    virtual mraa::Result write(const uint8_t* data, int length) = 0;
    virtual int read(uint8_t* data, int length) = 0;
  };

  class MraaI2c : public I2cInterface
  {
  public:
    MraaI2c(int bus, uint8_t address);
    mraa::Result write(const uint8_t* data, int length);
    int read(uint8_t* data, int length);
  private:
    mraa::I2c *i2c;
    int bus;
    uint8_t address;
  };


  class Dynamixel
  {
  public:
    explicit Dynamixel(uint8_t i2c_address, std::string frame_id, std::string child_frame_id);
    explicit Dynamixel(uint8_t i2c_address);
    explicit Dynamixel(I2cInterface *interface);
    explicit Dynamixel(I2cInterface *interface, std::string frame_id, std::string child_frame_id);

    void writeI2c(CLMessage32* message);
    void readI2c(CLMessage32* message);
    void setPosition(uint16_t position);
    void setVelocity(uint16_t velocity);
    void setPollingDt(uint16_t polling_dt);
    void getPosition(uint16_t *position);
    uint16_t getCurrentPosition();

    tf2::Transform getTransform();
    tf2::Stamped<tf2::Transform> getStampedTransform();
    geometry_msgs::TransformStamped getTransformMsg();
    void adjustCamera(int16_t velocity);
    void updatePosition();
    void scan();
    int16_t calculateDesiredVelocity(double theta);

    static uint8_t computeChecksum(CLMessage32 message);

    void reconfigureCallback(dynamixel::DynamicServoConfig &config, uint32_t level);

  private:
    I2cInterface *i2c;
    uint16_t current_position;
    double max_velocity;
    double v_s;

    tf2::Transform transform;
    ros::Time stamp;
    unsigned int seq;
    std::string frame_id;
    std::string child_frame_id;

    double current_velocity;
    bool track_tag;
  };
}


#endif //DYNAMIXEL_HOST_LAYER_H
