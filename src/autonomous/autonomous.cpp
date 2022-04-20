#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/highLift.hpp"
#include "subsystems/powershare.hpp"
#include "autonomous/autonomous.hpp"

#define kPneumaticClampPort 0
#define kPneumaticTilterPort 0
#define kPneumaticTransmissionPort 0

// functions
void highLiftTask(void* ignore) {
  continueHighLift = true;
  highTogglePosition();
}
void giveUp() {
  continueHighLift = true;
  while (continueHighLift) {
    if (chassis->getState().x.convert(okapi::foot) >= 5.5) {
      pros::c::adi_digital_write(kPneumaticClampPort, HIGH);
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
