#pragma once
#include "main.h"

// variables
// port numbers
extern const int8_t kDriveLBPort;
extern const int8_t kDriveLMPort;
extern const int8_t kDriveLTPort;
extern const int8_t kDriveRBPort;
extern const int8_t kDriveRMPort;
extern const int8_t kDriveRTPort;

// chassis
extern std::shared_ptr<okapi::OdomChassisController> chassis;

// motors
extern okapi::Motor driveLBMotor;
extern okapi::Motor driveLFMotor;
extern okapi::Motor driveRBMotor;
extern okapi::Motor driveRFMotor;


// functions
// none
