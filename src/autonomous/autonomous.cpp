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

/*
pros::c::ext_adi_digital_write(2, kPneumaticClampPort, LOW);
pros::c::adi_digital_write(kPneumaticTilterPort, LOW);
pros::c::ext_adi_digital_write(2, kPneumaticTilterPort2, HIGH);
pros::c::adi_digital_write(kPneumaticTransmissionPort, LOW);
pros::c::adi_digital_write(kPneumaticCoverPort, LOW);
*/

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
  pros::c::ext_adi_digital_write(2, kPneumaticTilterPort2, LOW);
}
void untilt() {
  pros::c::ext_adi_digital_write(2, kPneumaticTilterPort2, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticTilterPort, LOW);
}
void place() {
}

void right() {
}
void rightOne() {
  // pros::c::adi_digital_write(kPneumaticCoverPort, LOW);
  jCurve(3.8, 0, true, 0, 1, 2);
  pros::c::ext_adi_digital_write(2, kPneumaticClampPort, HIGH);
  pros::delay(250);
  jCurve(0, 0, false);

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
