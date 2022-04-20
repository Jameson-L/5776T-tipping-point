#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/highLift.hpp"
#include "subsystems/powershare.hpp"
#include "autonomous/autonomous.hpp"

#define kPneumaticClampPort 7
#define kPneumaticTilterPort 6
#define kPneumaticTransmissionPort 3

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
