#pragma once
#include "main.h"

// variables
// portNumbers
extern uint8_t kLTrackingWheelPortA;
extern uint8_t kLTrackingWheelPortB;
extern uint8_t kRTrackingWheelPortA;
extern uint8_t kRTrackingWheelPortB;
extern uint8_t kMTrackingWheelPortA;
extern uint8_t kMTrackingWheelPortB;

extern uint8_t imu1Port;
extern uint8_t imu2Port;

// odometry chassis declared in chassis.hpp

// IMUs
extern okapi::IMU imu1;
extern okapi::IMU imu2;

extern okapi::ADIButton bumper;

// distance Sensors
extern okapi::DistanceSensor LBDistanceSensor;
extern okapi::DistanceSensor LFDistanceSensor;

// ADI Encoders
extern okapi::ADIEncoder LTrackingWheel;
extern okapi::ADIEncoder RTrackingWheel;
extern okapi::ADIEncoder MTrackingWheel;

// drive PIDs
extern okapi::IterativePosPIDController chassisTurnPid;
extern okapi::IterativePosPIDController chassisDrivePid;
extern okapi::IterativePosPIDController chassisAnglePid;
extern okapi::IterativePosPIDController chasissSwingPid;
extern okapi::IterativePosPIDController chasissClimbPid;
extern okapi::IterativePosPIDController chasissVisionPid;

// functions
// helper functions
double getHeading(bool safe = false);
bool isMoving();

void odomDriveToPoint(double x, double y, bool forward=true, double offset = 0.0, double speedMultiplier = 1, double time = 4, bool persist = false, bool rush = false, int useVision = 0);
void jCurve(double x, double y, bool forward=true, double offset = 0.0, double speedMultiplier = 1, double time = 4, bool persist = false, bool rush = false, int useVision = 0);
void imuTurnToAngle(double deg);
void imuZeroToAngle(double deg, double time = 2);
void relative(double x, double time = 2);
void climb();
void visionAlign();
extern okapi::ADIButton bumper;
extern pros::vision_signature_s_t NEUTRAL;
extern pros::vision_signature_s_t RED;
extern pros::vision_signature_s_t BLUE;
extern pros::Vision vision;
extern pros::Vision vision2;
