#pragma once
#include "main.h"

// variables
// port numbers

extern const int8_t kHighLiftPort;
extern const int8_t kHighLiftLPort;
extern const int8_t kHighLiftRPort;

extern const int8_t kHighLiftLPotPort;
extern const int8_t kHighLiftRPotPort;

extern const int8_t kHighLiftDownLimitPort;
extern const int8_t kHighLiftUpLimitPort;

// position targets
extern const double kHighLiftDownTarget;
extern const double kHighLiftHoldTarget;
extern const double kHighLiftUpTarget;
extern const double kHighLiftMaxTarget;
extern const double kHighLiftMidTarget;

// motors
extern okapi::Motor highLiftLMotor;
extern okapi::Motor highLiftRMotor;

// motor group
extern okapi::Motor highLift;
// extern okapi::MotorGroup highLift;

// potentiometers
extern okapi::Potentiometer highLiftLPot;
extern okapi::Potentiometer highLiftRPot;


// high lift PIDs
extern okapi::IterativePosPIDController highLiftPid;

extern bool continueHighLift;
extern int state;
extern okapi::MedianFilter<10> highLiftFilter;

// functions
void highTogglePosition();
