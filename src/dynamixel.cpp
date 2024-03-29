#include <dynamixel.h>
#include <comm_layer_defs.h>
#include <iostream>

namespace dynamixel
{

Dynamixel::Dynamixel(uint8_t i2c_address) : Dynamixel(new MraaI2c(0, i2c_address))
{}

Dynamixel::Dynamixel(uint8_t i2c_address, std::string frame_id, std::string child_frame_id) :
    Dynamixel(new MraaI2c(0, i2c_address), frame_id, child_frame_id)
{}

Dynamixel::Dynamixel(uint8_t i2c_address, std::string frame_id, std::string child_frame_id,
                     uint16_t min, uint16_t max, uint16_t offset, double max_velocity) :
    Dynamixel(new MraaI2c(0, i2c_address), frame_id, child_frame_id, min, max, offset, max_velocity)
{}


Dynamixel::Dynamixel(I2cInterface *interface) : Dynamixel(interface, "servo_base_link", "servo_joint")
{}

Dynamixel::Dynamixel(I2cInterface *interface, std::string frame_id, std::string child_frame_id) :
    Dynamixel(interface, frame_id, child_frame_id, 12, 1011, 0, 0.8)
{}

Dynamixel::Dynamixel(I2cInterface *interface, std::string frame_id, std::string child_frame_id,
                     uint16_t min, uint16_t max, uint16_t offset, double max_velocity)
{
  this->track_tag = false;
  this->max_velocity = max_velocity;
  this->min = min;
  this->max = max;
  this->offset = offset;

  this->i2c = interface;
  seq = 0;
  //transform.setOrigin(tf2::Vector3(24.15e-3, 0.0, 32.5e-3));
  transform.setOrigin(tf2::Vector3(0.016256, 0.000508, 0.0195072));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  this->frame_id = frame_id;
  this->child_frame_id = child_frame_id;

  v_s = 86.0297; // Conversion from rad/s to servo units
  current_position = 512;
  current_velocity = 0;
}

void Dynamixel::reconfigureCallback(DynamicServoConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request - Max Velocity: %f, Track Tag: %s",
           config.max_velocity, config.track_tag ? "true" : "false");
  this->max_velocity = config.max_velocity;
  this->track_tag = config.track_tag;
  if (level == 1 && !this->track_tag) // TODO make this a set point instead
  {
    ROS_INFO("Setting position to: %i", config.set_position);
    this->setVelocity(50);
    this->setPosition((uint16_t) config.set_position);
  }
}

uint8_t Dynamixel::computeChecksum(CLMessage32 message)
{
  uint8_t checksum = 0;
  for (int i = 0; i < 3; i++)
  {
    checksum += message.data8[i];
  }
  return ~checksum;
}

void Dynamixel::writeI2c(CLMessage32 *message)
{
  if (i2c->write(message->data8, 4) != mraa::SUCCESS)
  {
    throw cl_tx_error(" I2C TX Error");
  }
}

void Dynamixel::readI2c(CLMessage32 *message)
{
  if (i2c->read(message->data8, 4) != 4)
  {
    throw cl_rx_error("[readI2c] I2C RX Error");
  }
  if (message->ucl.instruction == CL_ERROR) // TODO change to something more descriptive
  {
    throw cl_device_error("[readI2c] Device returned error"); // TODO maybe move these to the general methods?
  }
  if (computeChecksum(*message) != message->ucl.checksum)
  {
    throw cl_checksum_error("[readI2c] Checksum error");
  }
}

//TODO add error checking for both setPosition and getPosition
void Dynamixel::setPosition(uint16_t position)
{
  if (position + offset > 1023)
  {
    throw cl_error("[setPosition] desired position is out of bounds");
  }
  CLMessage32 message;
  message.ucl.instruction = DYN_SET_POSITION;
  message.ucl.data = position + offset;
  message.ucl.checksum = computeChecksum(message);
  writeI2c(&message);
  readI2c(&message);
}

// TODO check for errors on read
void Dynamixel::setVelocity(uint16_t velocity)
{
  if (velocity > 1023)
  {
    throw cl_error("[setVelocity] desired velocity is out of bounds (expected velocity = 0-1023)");
  }
  CLMessage32 message;
  message.ucl.instruction = DYN_SET_VELOCITY;
  message.ucl.data = velocity;
  message.ucl.checksum = computeChecksum(message);
  writeI2c(&message);
  readI2c(&message);
}

// TODO check for errors on read, make sure polling dt is reasonable
void Dynamixel::setPollingDt(uint16_t polling_dt)
{
  CLMessage32 message;
  message.ucl.instruction = DYN_SET_POLLING_DT;
  message.ucl.data = polling_dt;
  message.ucl.checksum = computeChecksum(message);
  writeI2c(&message);
  readI2c(&message);
}

void Dynamixel::getPosition(uint16_t *position)
{
  CLMessage32 message;
  message.ucl.instruction = DYN_GET_POSITION_INSTRUCTION;
  message.ucl.data = 0;
  message.ucl.checksum = computeChecksum(message);
  writeI2c(&message);
  readI2c(&message);
  if (message.ucl.data > 1023)
  {
    throw cl_device_error("[getPosition] Device returned out of bounds position (not 0-1023)");
  }
  *position = message.ucl.data - offset;
}

uint16_t Dynamixel::getCurrentPosition()
{
  return current_position;
}

int16_t Dynamixel::calculateDesiredVelocity(double theta)
{
  // Adjust servo
  double mx_v = max_velocity;
  double mn_v = 0.0;
  double fov = 0.542797; // 62.2 / 180 / 2 * pi
  double deadzone = 0.01745 * 3; // one degree / 2

  if (fabs(theta) <= deadzone)
  {
    return 0;
  }

  double sign = (theta < 0.0 ? -1 : 1);
  theta = fabs(theta);
  double v_r = sign * mx_v * theta / fov; // Desired servo velocity
  //printf("v_s*v_rs: %f\n", v_s*v_r);
  return (int16_t) (v_s * v_r);
}

void Dynamixel::updatePosition()
{
  getPosition(&(this->current_position));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0,
           ((double) current_position - 512.0) * (M_PI * 150.0) / (180.0 * 512.0)); // TODO make sure this is correct
  transform.setRotation(q);
  stamp = ros::Time::now();
  seq++;
}

void Dynamixel::adjustCamera(int16_t velocity)
{
  // Send control message
  CLMessage32 message;
  message.cl.instruction = DYN_ADJUST_SERVO;
  message.cl.data = velocity;
  message.cl.checksum = computeChecksum(message);
  writeI2c(&message);
  readI2c(&message);
  this->current_velocity = ((double) velocity) / v_s;
}

void Dynamixel::scan()
{
  // Send control message
  double velocity = 0.0;

  // If we aren't moving, pick the direction that covers the most area
  if (current_velocity == 0)
  {
    if (current_position <= 512)
    {
      velocity = max_velocity;
    }
    else
    {
      velocity = -max_velocity;
    }
  }

    // If we are moving, continue moving in the same direction but faster
    // TODO maybe add a ramp here?
  else if (current_velocity > 0)
  {
    velocity = max_velocity;
  }
  else if (current_velocity < 0)
  {
    velocity = -max_velocity;
  }

  // If we've hit the end, change direction
  if (current_position >= max)
  {
    velocity = -max_velocity;
  }
  if (current_position <= min)
  {
    velocity = max_velocity;
  }

  adjustCamera(velocity * v_s / 2); // TODO bad
}

tf2::Transform Dynamixel::getTransform()
{
  return this->transform;
}

tf2::Stamped<tf2::Transform> Dynamixel::getStampedTransform()
{
  tf2::Stamped<tf2::Transform> transform(this->transform, stamp,
                                         frame_id); // TODO probably should just do this right away
  return transform;
}

geometry_msgs::TransformStamped Dynamixel::getTransformMsg()
{
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.transform = tf2::toMsg(transform);
  transform_msg.header.frame_id = frame_id;
  transform_msg.child_frame_id = child_frame_id;
  transform_msg.header.seq = seq;
  transform_msg.header.stamp = stamp;
  return transform_msg;
}

MraaI2c::MraaI2c(int bus, uint8_t address)
{
  if (mraa_get_platform_type() == MRAA_UNKNOWN_PLATFORM || mraa_get_platform_type() == MRAA_NULL_PLATFORM)
  {
    throw cl_host_error("Host does not support mraa::i2c");
  }
  this->i2c = new mraa::I2c(bus);
  this->bus = bus;
  this->address = address;
}

mraa::Result MraaI2c::write(const uint8_t *data, int length)
{
  i2c->address(this->address);
  return i2c->write(data, length);
}

int MraaI2c::read(uint8_t *data, int length)
{
  i2c->address(this->address);
  return i2c->read(data, length);
}

}

