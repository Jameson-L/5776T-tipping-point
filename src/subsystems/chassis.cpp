#include "subsystems/chassis.hpp"
#include "autonomous/odometry.hpp"

// units
using namespace okapi::literals;

// defining chassis ports, negative is reversed
const int8_t kDriveLBPort = -6;
const int8_t kDriveLMPort = 8;
const int8_t kDriveLTPort = -9;
const int8_t kDriveRBPort = 3; 
const int8_t kDriveRMPort = -4;
const int8_t kDriveRTPort = 5;

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
