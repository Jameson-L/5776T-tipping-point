#include "main.h"
#include "subsystems/highLift.hpp"

// variables
// port numbers

// const int8_t kHighLiftPort = 2;
const int8_t kHighLiftLPort = -19;
// const int8_t kHighLiftRPort = -18;

const int8_t kHighLiftLPotPort = 0; // 1
// const int8_t kHighLiftRPotPort = 1;

// position targets
const double kHighLiftDownTarget = 940; // 0
const double kHighLiftHoldTarget = 1050; // 3
const double kHighLiftUpTarget = 1690; // 2
const double kHighLiftMaxTarget = 2000; // 4
const double kHighLiftMidTarget = 1350; // 1

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
			// highLiftPidValue = highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.15 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		} else if (state == 0) {
			// down
			highLiftPid.setTarget(kHighLiftDownTarget);
			// highLiftPidValue = highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.1 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		} else if (state == 3) {
      // kinda down
			highLiftPid.setTarget(kHighLiftHoldTarget);
			// highLiftPidValue = highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
      if (highLiftLPot.controllerGet() <= kHighLiftHoldTarget - 100) {
				highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.09 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
				highLiftPidValue *= 3.5;
			} else {
				highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.09 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
			}
		} else if (state == 1){
			// placing height
			highLiftPid.setTarget(kHighLiftMidTarget);
			// highLiftPidValue = highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.15 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		} else if (state == 5){
      highLiftPid.setTarget(kHighLiftMidTarget+240);
      highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.15 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
    } else {
      highLiftPid.setTarget(kHighLiftMaxTarget);
      highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.15 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
    }
      highLift.controllerSet(highLiftPidValue);
    }
    highLift.controllerSet(0);
    rate.delay(100_Hz);
}
