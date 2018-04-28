#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <dynamixel.h>
#include <comm_layer_defs.h>

void getRPY(tf2::Quaternion q, double &roll, double &pitch, double &yaw)
{
  tf2::Matrix3x3 matrix;
  matrix.setRotation(q);
  matrix.getRPY(roll, pitch, yaw);
}

using namespace dynamixel;
using testing::_;

class MockI2c : public I2cInterface
{
public:
  MOCK_METHOD2(write, mraa::Result(const uint8_t* data, int length));
  MOCK_METHOD2(read, int(uint8_t* data, int length));
};

TEST(DynamixelHostLayerTests, MockWriteTest)
{
  MockI2c test;
  uint8_t data[4] = {0, 1, 2, 3};
  EXPECT_CALL(test, write(data, 4))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  test.write(data, 4);
}

TEST(DynamixelHostLayerTests, ReadTest)
{
  MockI2c test;
  uint8_t data[4] = {0, 1, 2, 3};
  EXPECT_CALL(test, read(data, 4))
      .Times(1)
      .WillOnce(testing::Return(4));
  test.read(data, 4);
}

TEST(DynamixelHostLayerTests, Constructor)
{
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, 4)).Times(1).WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c,  read(_, 4)).Times(1).WillOnce(testing::Return(4));

  ros::Time::init();
  Dynamixel servo(&i2c);
  CLMessage32 message;
  message.ucl.instruction = DYN_SET_POSITION;
  message.ucl.data = 0;
  message.ucl.checksum = 0xE9;
  servo.writeI2c(&message);
  servo.readI2c(&message);
  geometry_msgs::TransformStamped tf = servo.getTransformMsg();
  ASSERT_STREQ(tf.header.frame_id.c_str(), "servo_base_link");
  ASSERT_STREQ(tf.child_frame_id.c_str(), "servo_joint");
}

TEST(DynamixelHostLayerTests, ComputeChecksum)
{
  CLMessage32 message;
  message.ucl.instruction = DYN_SET_POSITION;
  message.ucl.data = 12;
  message.ucl.checksum = Dynamixel::computeChecksum(message);
  ASSERT_EQ(message.ucl.checksum, 0xDD);
}

TEST(DynamixelHostLayerTests, WriteI2c)
{
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, 4))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  ros::Time::init();
  Dynamixel servo(&i2c);
  CLMessage32 message;
  servo.writeI2c(&message);
}

TEST(DynamixelHostLayerTests, WriteI2cError)
{
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, 4))
      .Times(1)
      .WillOnce(testing::Return(mraa::ERROR_UNSPECIFIED));
  ros::Time::init();
  Dynamixel servo(&i2c);
  CLMessage32 message;
  ASSERT_THROW(servo.writeI2c(&message), cl_tx_error);
}

TEST(DynamixelHostLayerTests, ReadI2c) // TODO add embedded system specific errors
{
  MockI2c i2c;
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(testing::Return(4));
  ros::Time::init();
  Dynamixel servo(&i2c);
  CLMessage32 message;
  message.ucl.instruction = DYN_SET_POSITION;
  message.ucl.data = 0;
  message.ucl.checksum = 0xE9;
  servo.readI2c(&message);
}

TEST(DynamixelHostLayerTests, ReadI2cWrongReturnValue) // TODO add embedded system specific errors
{
  MockI2c i2c;
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(testing::Return(1));
  ros::Time::init();
  Dynamixel servo(&i2c);
  CLMessage32 message;
  ASSERT_THROW(servo.readI2c(&message), cl_rx_error);
}

TEST(DynamixelHostLayerTests, ReadI2cDeviceError) // TODO add embedded system specific errors
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  MockI2c i2c;
  uint8_t error_message[4] = {CL_ERROR, 0, 0, 0xFE};
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(
          DoAll(
              SetArrayArgument<0>(
                  error_message, error_message + 4),
              Return(4)));
  ros::Time::init();
  Dynamixel servo(&i2c);
  CLMessage32 message;
  ASSERT_THROW(servo.readI2c(&message), cl_device_error);
}

TEST(DynamixelHostLayerTests, ReadI2cChecksumError) // TODO add embedded system specific errors
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  MockI2c i2c;
  uint8_t error_message[4] = {CL_OK, 0, 0, 0x01};
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(
          DoAll(
              SetArrayArgument<0>(error_message, error_message + 4),
              Return(4)));
  ros::Time::init();
  Dynamixel servo(&i2c);
  CLMessage32 message;
  ASSERT_THROW(servo.readI2c(&message), cl_checksum_error);
}

TEST(DynamixelHostLayerTests, SetPositionNormal)
{
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_SET_POSITION, 0xE8, 0x03, 0xFE))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, _))
      .With(testing::ElementsAre(DYN_SET_POSITION, 0xE8, 0x03, 0xFE))
      .Times(1)
      .WillOnce(testing::Return(4));

  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.setPosition(1000);
}

TEST(DynamixelHostLayerTests, SetPositionEdgeCaseIn)
{
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_SET_POSITION, 0xFF, 0x03, 0xE7))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, _))
      .With(testing::ElementsAre(DYN_SET_POSITION, 0xFF, 0x03, 0xE7))
      .Times(1)
      .WillOnce(testing::Return(4));

  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.setPosition(1023);
}

TEST(DynamixelHostLayerTests, SetPositionEdgeCaseOut)
{
  MockI2c i2c;
  ros::Time::init();
  Dynamixel servo(&i2c);
  ASSERT_THROW(servo.setPosition(1024), cl_error);
}

TEST(DynamixelHostLayerTests, SetVelocityNormal)
{
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_SET_VELOCITY, 0xE8, 0x03, 0xFB))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, _))
      .With(testing::ElementsAre(DYN_SET_VELOCITY, 0xE8, 0x03, 0xFB))
      .Times(1)
      .WillOnce(testing::Return(4));

  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.setVelocity(1000);
}

TEST(DynamixelHostLayerTests, SetVelocityEdgeCaseIn)
{
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_SET_VELOCITY, 0xFF, 0x03, 0xE4))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, _))
      .With(testing::ElementsAre(DYN_SET_VELOCITY, 0xFF, 0x03, 0xE4))
      .Times(1)
      .WillOnce(testing::Return(4));

  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.setVelocity(1023);
}

TEST(DynamixelHostLayerTests, SetVelocityEdgeCaseOut)
{
  MockI2c i2c;
  ros::Time::init();
  Dynamixel servo(&i2c);
  ASSERT_THROW(servo.setVelocity(1024), cl_error);
}

TEST(DynamixelHostLayerTests, SetPollingDt)
{
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_SET_POLLING_DT, 0x58, 0x00, 0x82))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, _))
      .With(testing::ElementsAre(DYN_SET_POLLING_DT, 0x58, 0x00, 0x82))
      .Times(1)
      .WillOnce(testing::Return(4));

  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.setPollingDt(88);
}

TEST(DynamixelHostLayerTests, GetPosition)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_GET_POSITION_INSTRUCTION, 0x00, 0x00, 0xE8))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  uint8_t return_message[4] = {DYN_GET_POSITION_RESPONSE, 0xE8, 0x03, 0xFC};
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(
          DoAll(
              SetArrayArgument<0>(return_message, return_message + 4),
              Return(4)));
  ros::Time::init();
  Dynamixel servo(&i2c);
  uint16_t position;
  servo.getPosition(&position);
  ASSERT_EQ(position, 1000);
}

TEST(DynamixelHostLayerTests, GetPositionEdgeCaseInBounds)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_GET_POSITION_INSTRUCTION, 0x00, 0x00, 0xE8))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  uint8_t return_message[4] = {DYN_GET_POSITION_RESPONSE, 0xFF, 0x03, 0xE5};
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(
          DoAll(
              SetArrayArgument<0>(return_message, return_message + 4),
              Return(4)));
  ros::Time::init();
  Dynamixel servo(&i2c);
  uint16_t position;
  servo.getPosition(&position);
  ASSERT_EQ(position, 1023);
}


TEST(DynamixelHostLayerTests, GetPositionEdgeCaseOutOfBounds)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_GET_POSITION_INSTRUCTION, 0x00, 0x00, 0xE8))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  uint8_t return_message[4] = {DYN_GET_POSITION_RESPONSE, 0x00, 0x04, 0xE3};
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(
          DoAll(
              SetArrayArgument<0>(return_message, return_message + 4),
              Return(4)));
  ros::Time::init();
  Dynamixel servo(&i2c);
  uint16_t position;
  ASSERT_THROW(servo.getPosition(&position), cl_device_error);
}

TEST(DynamixelHostLayerTests, UpdatePositionMiddle)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_GET_POSITION_INSTRUCTION, 0x00, 0x00, 0xE8))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  uint8_t return_message[4] = {DYN_GET_POSITION_RESPONSE, 0x00, 0x02, 0xE5};
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(
          DoAll(
              SetArrayArgument<0>(return_message, return_message + 4),
              Return(4)));
  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.updatePosition();
  tf2::Transform message = servo.getTransform();
  double roll, pitch, yaw;
  getRPY(message.getRotation(), roll, pitch, yaw);
  ASSERT_NEAR(roll,  0.0, 1E-10);
  ASSERT_NEAR(pitch, 0.0, 1E-10);
  ASSERT_NEAR(yaw,   0.0, 1E-10);
}

TEST(DynamixelHostLayerTests, UpdatePositionFarLeft)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_GET_POSITION_INSTRUCTION, 0x00, 0x00, 0xE8))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  uint8_t return_message[4] = {DYN_GET_POSITION_RESPONSE, 0xFF, 0x03, 0xE5};
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(
          DoAll(
              SetArrayArgument<0>(return_message, return_message + 4),
              Return(4)));
  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.updatePosition();
  tf2::Transform message = servo.getTransform();
  double roll, pitch, yaw;
  getRPY(message.getRotation(), roll, pitch, yaw);
  ASSERT_NEAR(roll,   0.0, 1E-10);
  ASSERT_NEAR(pitch,  0.0, 1E-10);
  ASSERT_NEAR(yaw,   2.62, 1E-2);
}

TEST(DynamixelHostLayerTests, UpdatePositionFarRight)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_GET_POSITION_INSTRUCTION, 0x00, 0x00, 0xE8))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  uint8_t return_message[4] = {DYN_GET_POSITION_RESPONSE, 0x00, 0x00, 0xE7};
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(
          DoAll(
              SetArrayArgument<0>(return_message, return_message + 4),
              Return(4)));
  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.updatePosition();
  tf2::Transform message = servo.getTransform();
  double roll, pitch, yaw;
  getRPY(message.getRotation(), roll, pitch, yaw);
  ASSERT_NEAR(roll,    0.0, 1E-10);
  ASSERT_NEAR(pitch,   0.0, 1E-10);
  ASSERT_NEAR(yaw,   -2.62, 1E-2);
}

TEST(DynamixelHostLayerTests, AdjustCamera)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  MockI2c i2c;
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 0x64, 0x00, 0x75))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  uint8_t return_message[4] = {DYN_ADJUST_SERVO, 0x00, 0x00, 0xD9};
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(
          DoAll(
              SetArrayArgument<0>(return_message, return_message + 4),
              Return(4)));
  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.adjustCamera(100);
}

TEST(DynamixelHostLayerTests, ScanInitPositive)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  using testing::InSequence;

  InSequence setup;

  MockI2c i2c;
  uint8_t scan_rx_message[4] = {DYN_ADJUST_SERVO, 0x00, 0x00, 0xD9};

  // First scan call
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 50, 0x00, 0xA7))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
              SetArrayArgument<0>(scan_rx_message, scan_rx_message + 4),
              Return(4)));

  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.scan();
}

TEST(DynamixelHostLayerTests, ScanInitNegative)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  using testing::InSequence;

  InSequence setup;

  MockI2c i2c;
  uint8_t update_position_message[4] = {DYN_GET_POSITION_RESPONSE, 0x01, 0x02, 0xE4};
  uint8_t scan_rx_message[4]         = {DYN_ADJUST_SERVO,          0x00, 0x00, 0xD9};

  // Get initial position
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_GET_POSITION_INSTRUCTION, 0x00, 0x00, 0xE8))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
              SetArrayArgument<0>(update_position_message, update_position_message + 4),
              Return(4)));

  // First scan call
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 0xCE, 0xFF, 0x0C))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(scan_rx_message, scan_rx_message + 4),
          Return(4)));

  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.updatePosition();
  servo.scan();
}

TEST(DynamixelHostLayerTests, ScanSaturateVelocityPositive)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  using testing::InSequence;

  InSequence setup;

  MockI2c i2c;
  uint8_t update_velocity_message[4] = {DYN_ADJUST_SERVO, 0x10, 0x00, 0xC9};
  uint8_t scan_rx_message[4]         = {DYN_ADJUST_SERVO, 0x00, 0x00, 0xD9};

  // Set initial velocity
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 0x10, 0x00, 0xC9))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(update_velocity_message, update_velocity_message + 4),
          Return(4)));

  // First scan call
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 0x32, 0x00, 0xA7))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(scan_rx_message, scan_rx_message + 4),
          Return(4)));

  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.adjustCamera(0x10);
  servo.scan();
}

TEST(DynamixelHostLayerTests, ScanSaturateVelocityNegative)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  using testing::InSequence;

  InSequence setup;

  MockI2c i2c;
  uint8_t update_velocity_message[4] = {DYN_ADJUST_SERVO, 0xF6, 0xFF, 0xE4};
  uint8_t scan_rx_message[4]         = {DYN_ADJUST_SERVO, 0x00, 0x00, 0xD9};

  // Set initial velocity
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 0xF6, 0xFF, 0xE4))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(update_velocity_message, update_velocity_message + 4),
          Return(4)));

  // First scan call
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 0xCE, 0xFF, 0x0C))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(scan_rx_message, scan_rx_message + 4),
          Return(4)));

  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.adjustCamera(-10);
  servo.scan();
}

TEST(DynamixelHostLayerTests, Scan)
{
  using testing::DoAll;
  using testing::SetArrayArgument;
  using testing::Return;
  using testing::InSequence;

  InSequence setup;

  MockI2c i2c;
  uint8_t scan_rx_message[4]         = {DYN_ADJUST_SERVO, 0x00, 0x00, 0xD9};

  // Move in positive direction
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 50, 0x00, 0xA7))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(scan_rx_message, scan_rx_message + 4),
          Return(4)));

  // Keep moving
  uint8_t update_position_message_1[4] = {DYN_GET_POSITION_RESPONSE, 0xBC, 0x02, 0x29};
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_GET_POSITION_INSTRUCTION, 0x00, 0x00, 0xE8))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(update_position_message_1, update_position_message_1 + 4),
          Return(4)));

  // No change
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 50, 0x00, 0xA7))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(scan_rx_message, scan_rx_message + 4),
          Return(4)));

  // Reached end
  uint8_t update_position_message_2[4] = {DYN_GET_POSITION_RESPONSE, 0xFF, 0x03, 0xE5};
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_GET_POSITION_INSTRUCTION, 0x00, 0x00, 0xE8))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(update_position_message_2, update_position_message_2 + 4),
          Return(4)));

  // Reverse direction
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 0xCE, 0xFF, 0x0C))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(scan_rx_message, scan_rx_message + 4),
          Return(4)));

  // Keep moving
  uint8_t update_position_message_3[4] = {DYN_GET_POSITION_RESPONSE, 0x20, 0x03, 0xC4};
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_GET_POSITION_INSTRUCTION, 0x00, 0x00, 0xE8))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(update_position_message_3, update_position_message_3 + 4),
          Return(4)));

  // No change
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 0xCE, 0xFF, 0x0C))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(scan_rx_message, scan_rx_message + 4),
          Return(4)));

  // Reached the other end
  uint8_t update_position_message_4[4] = {DYN_GET_POSITION_RESPONSE, 0x00, 0x00, 0xE7};
  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_GET_POSITION_INSTRUCTION, 0x00, 0x00, 0xE8))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(update_position_message_4, update_position_message_4 + 4),
          Return(4)));

  EXPECT_CALL(i2c, write(_, _))
      .With(testing::ElementsAre(DYN_ADJUST_SERVO, 50, 0x00, 0xA7))
      .Times(1)
      .WillOnce(testing::Return(mraa::SUCCESS));
  EXPECT_CALL(i2c, read(_, 4))
      .Times(1)
      .WillOnce(DoAll(
          SetArrayArgument<0>(scan_rx_message, scan_rx_message + 4),
          Return(4)));

  ros::Time::init();
  Dynamixel servo(&i2c);
  servo.scan();            // Move in the positive direction
  servo.updatePosition();  // Keep moving
  servo.scan();            // No change
  servo.updatePosition();  // Reached end
  servo.scan();            // Reverse direction
  servo.updatePosition();  // Keep moving
  servo.scan();            // No change
  servo.updatePosition();  // Reached other end
  servo.scan();            // Reverse direction
}

/*
 * Example array tests
 */
class MockArrayFunc {
public:
  // Array passed by reference.
  MOCK_METHOD1(ArrayFunc1, void(const int (&array)[3]));
  // Array passed by pointer and size.
  MOCK_METHOD2(ArrayFunc2, void(const int* ptr, int size));
};

TEST(ElementsAreTest, Works) {
  MockArrayFunc f;
  int array[3] = { 1, 2, 3 };
  int array2[3] = { 2, 3, 4 };

  EXPECT_CALL(f, ArrayFunc1(testing::ElementsAreArray(array)));
  EXPECT_CALL(f, ArrayFunc1(testing::ElementsAre(2, 3, 4)));
  EXPECT_CALL(f, ArrayFunc2(_, _)).With(testing::ElementsAreArray(array));
  EXPECT_CALL(f, ArrayFunc2(_, _)).With(testing::ElementsAre(2, 3, 4));
  f.ArrayFunc1(array);
  f.ArrayFunc1(array2);
  f.ArrayFunc2(array, 3);
  f.ArrayFunc2(array2, 3);
}

int main(int argc, char **argv)
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}