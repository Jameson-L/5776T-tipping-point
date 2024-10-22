#include "main.h"
#include "subsystems/highLift.hpp"

// variables
// port numbers

// const int8_t kHighLiftPort = 2;
const int8_t kHighLiftLPort = 20;
// const int8_t kHighLiftRPort = -18;

const int8_t kHighLiftLPotPort = 5; // 1
// const int8_t kHighLiftRPotPort = 1;

// position targets
//733
const double kHighLiftDownTarget = 993; // 0
const double kHighLiftHoldTarget = 1203; // 3
const double kHighLiftUpTarget = 2208; // 2
const double kHighLiftMaxTarget = 2208; // 4
const double kHighLiftMidTarget = 1533; // 1

// motors
// okapi::Motor highLiftLMotor;
// okapi::Motor highLiftRMotor;

// motor group
// okapi::MotorGroup highLift = okapi::MotorGroup({okapi::Motor(kHighLiftLPort), okapi::Motor(kHighLiftRPort)});
okapi::Motor highLift = okapi::Motor(kHighLiftLPort);

// potentiometers
okapi::Potentiometer highLiftLPot = okapi::Potentiometer({2, kHighLiftLPotPort});
// okapi::Potentiometer highLiftRPot = okapi::Potentiometer(kHighLiftRPotPort);

// limit switches (maybe)
// okapi::ADIButton highLiftDownLimit;
// okapi::ADIButton highLiftUpLimit;

// high lift PIDs
okapi::IterativePosPIDController highLiftPid = okapi::IterativeControllerFactory::posPID(0.002, 0.0, 0.000);

bool continueHighLift = false;
int state = 0;
okapi::MedianFilter<10> highLiftFilter;
// functions
void highTogglePosition() {
  highLift.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  double highLiftPidValue = 0;
  okapi::Rate rate;
  while (continueHighLift) {
    if(state == 2) {
			// up
			highLiftPid.setTarget(kHighLiftUpTarget);
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.15 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		} else if (state == 0) {
			// down
			highLiftPid.setTarget(kHighLiftDownTarget);
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.1 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		} else if (state == 3) {
			// kinda down
			highLiftPid.setTarget(kHighLiftHoldTarget);
			if (highLiftLPot.controllerGet() <= kHighLiftHoldTarget-20) {
				highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.09 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
				highLiftPidValue *= 3;
			} else {
				highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.09 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
			}
		} else {
			// placing height
			highLiftPid.setTarget(kHighLiftMidTarget);
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.15 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		}
      highLift.controllerSet(highLiftPidValue);
    }
    highLift.controllerSet(0);
    rate.delay(100_Hz);
}
