#pragma once
#include "main.h"

// variables
// port numbers
extern const int8_t kConveyorPort;
// extern const int8_t kConveyorEncoderPort;

// motors
extern okapi::Motor conveyorMotor;

// ADI encoders (maybe)
// extern okapi::ADIEncoder conveyorEncoder;

// Conveyor PIDs (need to test)
extern okapi::IterativeVelPIDController conveyorVelPID;


// functions
void intake(double speed);
void outtake(double speed);
