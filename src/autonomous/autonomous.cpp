#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/highLift.hpp"
#include "subsystems/conveyor.hpp"
#include "autonomous/autonomous.hpp"

#define kPneumaticClawPort 7
#define kPneumaticLiftPort 2

bool taskRunning = false;
// functions
void highLiftTask(void* ignore) {
  continueHighLift = true;
  highTogglePosition();
}
void giveUp() {
  continueHighLift = true;
  while (continueHighLift) {
    if (chassis->getState().x.convert(okapi::foot) >= 5.5) {
      pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
    }
  }
}
void lift() {
  pros::delay(750);
  state = 2;
  conveyorMotor.controllerSet(-1);
}
void lift2() {
  pros::delay(500);
  state = 0;
}
void middleTask() {
  while (true) {
    if (chassis->getState().x.convert(okapi::foot) >= 4.9) {
      pros::c::adi_digital_write(kPneumaticClawPort, LOW);
      break;
    }
  }
}
void tilterTask() {
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
}
void liftDown() {
  while (true) {
    if (chassis->getState().x.convert(okapi::foot) >= 2.6) {
      state = 0;
      conveyorMotor.controllerSet(0);
      break;
    }
  }
}
void tilterToLift() {
  taskRunning = true;
  // pros::Task highLift(highLiftTask);
  pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
  pros::delay(1000);
  relative(1.8, 0.2);
  pros::delay(500);
  state = 0;
  conveyorMotor.controllerSet(0);
  if (getHeading(false) > 0) {
    imuTurnToAngle(getHeading(false) - 180);
  } else if (getHeading(false) < 0) {
    imuTurnToAngle(getHeading(false) + 180);
  } else {
    imuTurnToAngle(180);
  }
  relative(1.5, 0.7);
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  pros::delay(100);
  // state = 3;
  // continueHighLift = false;
  taskRunning = false;
}
void place() {
  // pros::Task highLift(highLiftTask);
  state = 1;
  pros::delay(800);
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  pros::delay(250);
  state = 2;
  pros::delay(250);
  // continueHighLift = false;
}

void right() {
  pros::Task highLift(highLiftTask);
  pros::Task giveUpTask(giveUp);
  state = 0; // lowerlift if not lowered
  jCurve(4, 0, true, 0, 1, 2, false, true, true); // first mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW); // clamp
  pros::delay(100);
  jCurve(0.8, 0, false, 0, 1, 1.5, true); // drive back
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH); // unclamp
  jCurve(-0.6, 0, false, 0, 1, 0.7); // drive back
  jCurve(3.6, -2.5, true, 0, 1, 2.5, false, true, true); // middle mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW); // clamp
  pros::delay(100);
  jCurve(0, -0.008, false, 0, 1, 2, true); // drive back
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH); // unclamp
  // odomDriveToPoint(-0.5, 0.14, false, 0, 1, 0.7); // drive back
  jCurve(1.1, 1.85, false, 0, 1, 1.5); // third mogo
  pros::c::adi_digital_write(kPneumaticLiftPort, LOW); // clamp
  pros::delay(250);
  state = 3;
  conveyorMotor.controllerSet(1); // conveyor on
  pros::delay(750);
  relative(2, 0.3);
  conveyorMotor.controllerSet(0);
  state = 0;
  imuTurnToAngle(-45);
  jCurve(2, 0.3, true, 0, 1, 1.5);
  pros::c::adi_digital_write(kPneumaticClawPort, LOW); // clamp
  // jCurve(0, 0.5, false);
}
void rightOne() {
  pros::Task highLift(highLiftTask);
  pros::Task giveUpTask(giveUp);
  state = 0; // lowerlift if not lowered
  jCurve(4, 0, true, 0, 1, 2, false, true, true); // first mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW); // clamp
  pros::delay(100);
  jCurve(0.9, 0, false, 0, 1, 1.5, true); // drive back
  imuTurnToAngle(-90);
  relative(-2, 1);
  pros::c::adi_digital_write(kPneumaticLiftPort, LOW);
  pros::delay(500);
  state = 2;
  conveyorMotor.controllerSet(1);
  jCurve(0.9, 1.1, true, 0, 1, 0.7);
  imuTurnToAngle(0);
  jCurve(5, 1.1, true, 0, 0.3, 4);
  conveyorMotor.controllerSet(0);
  state = 0;
  jCurve(0, 1.1, false, 0, 2); // drive back
}
void rightAllianceWP() {
  pros::Task highLift(highLiftTask);
  pros::Task giveUpTask(giveUp);
  state = 0; // lowerlift if not lowered
  jCurve(3.5, 0, true, 0.0, 1, 1.3); // first mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW); // clamp
  odomDriveToPoint(-1, 0, false, 0, 1, 2);
  imuTurnToAngle(90);
}
void left() {
  pros::Task highLift(highLiftTask);
  pros::Task giveUpTask(giveUp);
  state = 0;
  jCurve(3.5, 0.75, true, 0, 1, 1.5); // first mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  jCurve(-0.2, -0.8, false, 1, 1, 1.5, true); // drive back
  // imuTurnToAngle(-45);
  odomDriveToPoint(-0.27, 1, false, 0, 1, 0);
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  jCurve(-0.27, 1, false, 0, 1, 1.5);
  pros::c::adi_digital_write(kPneumaticLiftPort, LOW);
  pros::delay(500);
  state = 3;
  conveyorMotor.controllerSet(1);
  pros::Task down(liftDown);
  jCurve(3.5, 3.4, true, -0.3, 0.5, 3); // middle mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  state = 3;
  conveyorMotor.controllerSet(-1);
  jCurve(0, 0, false, 0, 1, 2.5, true); // drive back
  state = 0;
  conveyorMotor.controllerSet(0);
  pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
  imuTurnToAngle(-45);
  // jCurve(0.8, 0, true, 0, 1, 1);
  // pros::c::adi_digital_write(kPneumaticClawPort, LOW);
}
void leftOne() {
  pros::Task highLift(highLiftTask);
  pros::Task giveUpTask(giveUp);
  state = 0;
  jCurve(3.5, 0.75, true, 0, 1, 1.5); // first mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  jCurve(-0.2, -0.8, false, 1, 1, 1.5, true); // drive back
  // imuTurnToAngle(-45);
  odomDriveToPoint(-0.27, 1, false, 0, 1, 0);
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  jCurve(-0.27, 1, false, 0, 1, 1.5);
  pros::c::adi_digital_write(kPneumaticLiftPort, LOW);
  pros::delay(500);
  state = 3;
  conveyorMotor.controllerSet(1);
  pros::delay(1000);
  conveyorMotor.controllerSet(0);
  state = 0;
  pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
  relative(1);
}
void leftCounter() {
  pros::Task highLift(highLiftTask);
  state = 0;
  odomDriveToPoint(-0.27, 1, false, 0, 1, 0);
  jCurve(-0.27, 1, false, 0, 1, 1);
  pros::c::adi_digital_write(kPneumaticLiftPort, LOW);
  pros::delay(500);
  state = 3;
  conveyorMotor.controllerSet(1);
  pros::Task down(liftDown);
  jCurve(3.5, 3.4, true, -0.3, 0.5, 3); // middle mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  state = 3;
  conveyorMotor.controllerSet(-1);
  jCurve(1, 2, false, 0, 1, 1.5, true); // drive back
  state = 0;
  conveyorMotor.controllerSet(0);
  imuTurnToAngle(90);
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  jCurve(1, 1.2, false, 0, 1, 1, true); // drive back
  pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
  imuTurnToAngle(0);
  jCurve(5.5, 1.2, true, 0, 1, 2.5);
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  jCurve(0, 0, false, 0, 1);
}
void soloAWP() {
  pros::Task highLift(highLiftTask);
  state = 3;
  // odomDriveToPoint(-0.35, 0, false, 0, 1, 0.8);
  odomDriveToPoint(-0.27, 1, false, 0, 1, 0);
  jCurve(-0.27, 1, false, 0, 1, 1);
  pros::c::adi_digital_write(kPneumaticLiftPort, LOW);
  pros::delay(500);
  conveyorMotor.moveRelative(400, 600);
  pros::delay(500);
  pros::Task tilter(tilterTask);
  jCurve(1.3, 0, true, 0, 1, 1.2);
  // pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
  // pros::delay(500);
  imuTurnToAngle(-90);
  jCurve(1.4, 8.5, false, 0, 1, 3.5);
  // jCurve(1.33, 8.5, false, 0, 1, 1);
  pros::c::adi_digital_write(kPneumaticLiftPort, LOW);
  pros::delay(750);
  conveyorMotor.controllerSet(1);
  pros::Task down(liftDown);
  jCurve(4, 4.4, true, 0.15, 0.8, 2);
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  jCurve(1, 7, false, 0, 1);
  /*
  pros::delay(500);
  jCurve(1.3, 6, true, 0, 1);
  conveyorMotor.controllerSet(0);
  pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
  pros::delay(750);
  jCurve(1.3, 5, true);
  state = 0;
  */
}

void skills() {
  pros::Task highLift(highLiftTask);
  chassis->setState({1.33_ft, 9_ft, 0_deg});
  state = 0;
  jCurve(1.6 + 1.33, 9, true, 0, 1, 1); // drive forward
  odomDriveToPoint(1.3 + 1.33, 11, false, 0.5, 1, 1); // back into alliance mogo
  pros::c::adi_digital_write(kPneumaticLiftPort, LOW);
  pros::delay(500);
  jCurve(3.6+1.33, 9.4, true, 0.0, 1, 1.5); // right neutral mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  pros::delay(250);
  state = 2;
  conveyorMotor.controllerSet(1);
  // jCurve(3.5+2.35, 9.4, true, 0, 1, 0.8);
  jCurve(9.25, 6.5, true, 0.2, 1, 2.5); // far plat
  // imuTurnToAngle(-45);
  place(); // place
  conveyorMotor.controllerSet(-1);
  // imuZeroToAngle(-45, 0.5);
  relative(-2, 0.5);
  tilterToLift(); // tilter to fourbar
  state = 5;
  odomDriveToPoint(8.8, 7.25, true, 0, 0.8, 1.7); // plat
  conveyorMotor.controllerSet(0);
  // place(); // place
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  pros::delay(500);
  pros::Task down(lift2);
  jCurve(8, 11, false, 0, 1, 1.5); // top right corner
  jCurve(10.5, 8.6, true, 1, 1, 1.7); // top right alliance mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  pros::delay(100);
  pros::Task lift1(lift);
  jCurve(3.5, 9, false, 0, 1, 3); // drive towards close plat
  odomDriveToPoint(1.6, 7.2, true, 0, 1, 0); // plat
  jCurve(1.6, 7.2, true, 0.2, 1, 1.8);
  // imuZeroToAngle(-165, 0.5);
  place(); // place
  pros::Task dl(lift2);
  jCurve(3.2, 9, false, 0, 1, 1); // back up
  state = 0;
  conveyorMotor.controllerSet(0);
  // state = 6;
  // pros::delay(750);
  // imuTurnToAngle(-45);
  pros::Task middle(middleTask);
  odomDriveToPoint(8, 4.5, true, -1, 1, 1); // middle mogo
  jCurve(9, 3.5, true, 0.3, 0.8, 2.5);  // drive to corner
  // conveyorMotor.controllerSet(0);
  // state = 0;
  // pros::delay(750);
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  // pros::delay(750);
  jCurve(8.1, 4.5, false, 0, 1, 1);
  // relative(-0.8, 1); // back up
  // state = 3;
  pros::Task liftDown2(lift2);
  odomDriveToPoint(8.05, 2, false, 0.2, 1, 0);
  jCurve(8.05, 2, false, 0.5, 1, 1.5); // top left alliance mogo
  pros::c::adi_digital_write(kPneumaticLiftPort, LOW);
  state = 0;
  pros::delay(500);
  jCurve(5.6, 3.8, true, 0, 1, 2); // far neutral mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  // pros::delay(500);
  state = 5;
  conveyorMotor.controllerSet(1);
  jCurve(1.8, 6, true, 0, 1, 2.5); // close plat
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  pros::delay(500);
  conveyorMotor.controllerSet(-1);
  // imuTurnToAngle(135);
  // imuTurnToAngle(170);
  relative(-2, 0.7);
  tilterToLift(); // tilter to fourbar
  state = 5;
  odomDriveToPoint(1, 6.25, true, 0.8, 0.5, 1.7); // close plat
  conveyorMotor.controllerSet(0);
  // place();
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  pros::delay(500);
  pros::Task liftDown(lift2);
  jCurve(2, 2, false, 0, 1, 1.5); // bottom left corner
  jCurve(0, 4.7, true, 1, 1, 1.5); // bottom left alliance mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  pros::delay(100);
  pros::Task lift2(lift);
  jCurve(5, 3, false, 0, 1, 1.5);
  state = 5;
  odomDriveToPoint(10, 6.3, true, 0.2, 1, 0);
  jCurve(10, 6.3, true, 1.1, 1, 2.5);
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  pros::delay(200);
  relative(-2);
  conveyorMotor.controllerSet(0);
}

/*
void skills() {
  okapi::Timer timer;
  chassis->setState({-2.4_ft, 1_ft});
  // chassis->setState({-1.6_ft, 1_ft});
  // pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
  pros::Task highLift(highLiftTask);
  state = 0;
  // pros::delay(1250);
  // odomDriveToPoint(-2.4, 1, false, 0, 1, 1);
  pros::c::adi_digital_write(kPneumaticLiftPort, LOW); // clamp alliance mogo
  // conveyorMotor.controllerSet(1);
  pros::delay(250);
  jCurve(-3, 6, true, 2.8, 1, 2.5); // neutral
  pros::c::adi_digital_write(kPneumaticClawPort, LOW); // clamp neutral mogo
  conveyorMotor.controllerSet(0);
  pros::delay(250);
  state = 2;
  jCurve(-4.3, 8, true, 3.5, 0.7, 2.3); // plat
  imuTurnToAngle(80);
  state = 1;
  pros::delay(750);
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  state = 2;
  conveyorMotor.controllerSet(1);
  odomDriveToPoint(-4, 7.5, false, 0, 0.6, 1.2); // back up
  state = 3;
  // pros::Task conveyor(conveyorRun);
  odomDriveToPoint(-8.6, 7.5, true, 1.5, 0.3, 2.5); // pick up rings on the way
  odomDriveToPoint(-8.6, 7.5, true, -2, 0.5, 2); // pick up rings on the way
  conveyorMotor.controllerSet(-1);
  odomDriveToPoint(-8.6, 7.7, false, 0, 1, 1); // pick up rings on the way
  pros::delay(300);
  conveyorMotor.controllerSet(0);
  state = 0;
  odomDriveToPoint(-8.6, 6, true, 0.4, 1, 1); // far neutral mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW); // clamp neutral mogo
  state = 2;
  jCurve(-7, 3.5, true, 3, 1, 2); // home plat
  state = 1;
  pros::delay(750);
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  state = 2;
  odomDriveToPoint(-8.8, 6, false, 2.2); // back up
  state = 0;
  odomDriveToPoint(-5.5, 6, true, 0.65, 0.5, 1.5); // middle neutral
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  pros::delay(250);
  state = 2;
  conveyorMotor.controllerSet(1);
  jCurve(-5.9, 9, true, 6.5, 0.7, 1.6); // far plat
  state = 1;
  pros::delay(750);
  conveyorMotor.controllerSet(-1);
  pros::delay(750);
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  // odomDriveToPoint(-6, 6, false, 0, 0.5, 0.2);
  // pros::delay(500);
  conveyorMotor.controllerSet(0);
  // state = 2;
  // pros::delay(250);
  pros::Task middle(lift);
  odomDriveToPoint(-6.5, 7.7, false, 0, 1, 1); // back up
  state = 3;
  conveyorMotor.controllerSet(1);
  odomDriveToPoint(-9.5, 7.7, true, 0, 0.7, 3); // pick up rings on the way
  conveyorMotor.controllerSet(0);
  state = 0;
  odomDriveToPoint(-7.5, 11, true, 0.5, 0.5, 1.5); // top left alliance mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  // pros::delay(250);
  // state = 1;
  odomDriveToPoint(-9, 7.7, false, 0, 1, 1); // back up
  pros::delay(250);
  state = 2;
  conveyorMotor.controllerSet(1);
  odomDriveToPoint(-5, 2.8, true, -0.5, 0.7, 3); // other plat
  state = 1;
  pros::delay(250);
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  // pros::delay(300);
  state = 2;
  pros::delay(250);
  odomDriveToPoint(-6.5, 4, false, 0, 1, 0.7); // back up
  state = 0;
  conveyorMotor.controllerSet(0);
  odomDriveToPoint(-10, 2.5, true, 1, 1, 2); // top right alliance mogo
  pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  pros::delay(250);
  state = 2;
  conveyorMotor.controllerSet(1);
  odomDriveToPoint(-5.5, 9, true, -0.9, 0.7, 2.9); // far plat
  // odomDriveToPoint(-7, 8, true, 0, 1, 0.5);
  state = 1;
  pros::delay(250);
  conveyorMotor.controllerSet(-1);
  pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
  pros::delay(500);
  state = 2;
  pros::delay(500);
  odomDriveToPoint(-6.5, 9.2, false, 0, 0.5, 0.5); // toggle this if cringe final movement
  imuTurnToAngle(0);
  pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
  imuTurnToAngle(-90);
  conveyorMotor.controllerSet(0);
  state = 0;
  // odomDriveToPoint(-1, 9.2, false, 0, 1, 1.5);
  // state = 0;
  // odomDriveToPoint(0, 9.2, true, 0, 1, 2);
  // pros::c::adi_digital_write(kPneumaticClawPort, LOW);
  // pros::delay(250);
  // odomDriveToPoint(0, 1);
  // std::cout << timer.millis().convert(okapi::second) << "\n ";
}
*/
