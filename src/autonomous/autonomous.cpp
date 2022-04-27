#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/highLift.hpp"
#include "subsystems/powershare.hpp"
#include "autonomous/autonomous.hpp"

#define kPneumaticClampPort 7
#define kPneumaticTilterPort 1
#define kPneumaticTilterPort2 6
#define kPneumaticTransmissionPort 3
#define kPneumaticCoverPort 4

// functions
void highLiftTask(void* ignore) {
  continueHighLift = true;
  highTogglePosition();
}
void giveUp() {
  continueHighLift = true;
  while (continueHighLift) {
    if (chassis->getState().x.convert(okapi::foot) >= 5.5) {
      pros::c::ext_adi_digital_write(2, kPneumaticClampPort, HIGH);
    }
  }
}
void tilt() {
  pros::c::adi_digital_write(kPneumaticTilterPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticTilterPort2, HIGH);
}
void untilt() {
  pros::c::adi_digital_write(kPneumaticTilterPort2, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticTilterPort, HIGH);
}
void place() {
}

void right() {
}
void rightOne() {
}
void rightAllianceWP() {
}
void left() {
}
void leftOne() {
}
void leftCounter() {
}
void soloAWP() {
}
void skills() {
}
