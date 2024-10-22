#include "subsystems/powershare.hpp"

const int8_t kPowersharePort = -21;

okapi::Motor powershare = okapi::Motor(kPowersharePort);

okapi::Potentiometer powersharePot = okapi::Potentiometer(2);

okapi::IterativePosPIDController powersharePid = okapi::IterativeControllerFactory::posPID(0.002, 0, 0);
const int powershareTarget = 3300;
const int powershareTarget2 = 2030;

bool continueLowLift = false;
int state2 = 0;
okapi::MedianFilter<10> lowLiftFilter;

void lowToggle() {
  powershare.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  okapi::Rate rate;
  while (continueLowLift) {
    if (state2 == 1) {
      powershare.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
      powersharePid.setTarget(powershareTarget);
      powershare.controllerSet(powersharePid.step(lowLiftFilter.filter(powersharePot.controllerGet())));
    } else if (state2 == 2){
      powershare.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
      powershare.controllerSet(-1);
    } else if (state2 == 3){
      powershare.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
      if (powersharePot.controllerGet() > powershareTarget2+120) {
        powershare.controllerSet(-1);
      } else {
        powershare.controllerSet(0);
      }
    } else {
      powershare.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      powershare.controllerSet(0);
    }
  }
  powershare.controllerSet(0);
  rate.delay(100_Hz);
}
