#pragma once
#include "main.h"

// variables
// port numbers
extern const int8_t kPowersharePort;
// extern const int8_t kConveyorEncoderPort;

// motors
extern okapi::Motor powershare;

extern okapi::Potentiometer powersharePot;
extern okapi::IterativePosPIDController powersharePid;
extern const int powershareTarget;

extern bool continueLowLift;
extern int state2;
extern okapi::MedianFilter<10> lowLiftFilter;

// functions
void lowTogglePosition();
