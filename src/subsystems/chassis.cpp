#include "subsystems/chassis.hpp"
#include "autonomous/odometry.hpp"

// units
using namespace okapi::literals;

// defining chassis ports, negative is reversed
const int8_t kDriveLBPort = -11; // -11
const int8_t kDriveLMPort = 10; // 3
const int8_t kDriveLTPort = -16; // -10
const int8_t kDriveRBPort = 3; // 6
const int8_t kDriveRMPort = -9; // -19
const int8_t kDriveRTPort = 8; // 9

// creating logger
// auto logger = okapi::Logger::getDefaultLogger();

// creating chassis object
std::shared_ptr<okapi::OdomChassisController> chassis = okapi::ChassisControllerBuilder()
  .withMotors(
    {kDriveLBPort, kDriveLMPort, kDriveLTPort},
    {kDriveRBPort, kDriveRMPort, kDriveRTPort}
  )
  .withDimensions(okapi::AbstractMotor::gearset::blue, okapi::ChassisScales({4._in, 13._in}, okapi::imev5BlueTPR * 7./3.))
  .withSensors(LTrackingWheel, RTrackingWheel/*, MTrackingWheel*/)
  .withOdometry({{2.75_in, 7.25_in}, okapi::quadEncoderTPR})
  .buildOdometry();
