#include "subsystems/chassis.hpp"
#include "autonomous/odometry.hpp"

using namespace okapi::literals;

uint8_t kLTrackingWheelPortA = 1;
uint8_t kLTrackingWheelPortB = 2;
uint8_t kRTrackingWheelPortA = 3;
uint8_t kRTrackingWheelPortB = 4;
// uint8_t kMTrackingWheelPortA = 3;
// uint8_t kMTrackingWheelPortB = 4;

uint8_t imu1Port = 12;
uint8_t imu2Port = 0;

okapi::Rate rate;

okapi::ADIEncoder LTrackingWheel = okapi::ADIEncoder({2, kLTrackingWheelPortA, kLTrackingWheelPortB}, false);
okapi::ADIEncoder RTrackingWheel = okapi::ADIEncoder({2, kRTrackingWheelPortA, kRTrackingWheelPortB}, false);
// okapi::ADIEncoder MTrackingWheel = okapi::ADIEncoder(kMTrackingWheelPortA, kMTrackingWheelPortB, true);

okapi::IMU imu1 = okapi::IMU(imu1Port, okapi::IMUAxes::z);
okapi::IMU imu2 = okapi::IMU(imu2Port, okapi::IMUAxes::x);

pros::Vision vision1(15, pros::E_VISION_ZERO_CENTER);
pros::Vision vision2(15, pros::E_VISION_ZERO_CENTER);
pros::vision_signature_s_t NEUTRAL = pros::Vision::signature_from_utility(1, 1453, 1881, 1667, -4979, -4407, -4693, 3.000, 0);
pros::vision_signature_s_t RED = pros::Vision::signature_from_utility(2, 8363, 9547, 8955, -927, -543, -735, 6.300, 0);
pros::vision_signature_s_t BLUE = pros::Vision::signature_from_utility(3, -3225, -2877, -3051, 12239, 13155, 12697, 11.000, 0);

okapi::ADIButton bumper = okapi::ADIButton(1);

okapi::IterativePosPIDController chassisTurnPid = okapi::IterativeControllerFactory::posPID(0.05, 0.0, 0.001);
okapi::IterativePosPIDController chassisDrivePid = okapi::IterativeControllerFactory::posPID(0.55, 0.01, 0.02);
okapi::IterativePosPIDController chassisSwingPid = okapi::IterativeControllerFactory::posPID(0.25, 0.0, 0.0025);
okapi::IterativePosPIDController chassisClimbPid = okapi::IterativeControllerFactory::posPID(0.05, 0.0, 0.001);
okapi::IterativePosPIDController chassisVisionPid = okapi::IterativeControllerFactory::posPID(0.005, 0.0, 0.000/*0009*/); // 0.00009

double getHeading(bool safe) {
  if (!safe) {
    return (imu1.controllerGet() + imu1.controllerGet()) / 2.0;
  } else {
    return std::fmod((getHeading(false) + 360), 360);
  }
}

bool isMoving() {
  return abs(okapi::Motor(kDriveLBPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveLMPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveLTPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveRBPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveRMPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveRTPort).getActualVelocity()) > 24;
};

void imuTurnToAngle(double deg) {
  // okapi::MotorGroup allMotors({kDriveLTPort, kDriveLMPort, kDriveLBPort, kDriveRBPort, kDriveRMPort, kDriveRTPort});
  // allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  bool safe = deg < -150 || deg > 150;
  if (deg < -150) {
    deg += 360;
  }
  chassisTurnPid.setTarget(deg);

  double chassisPidValue;

  okapi::Timer timer;
  double init = timer.millis().convert(okapi::second);

  while (!(abs(deg - getHeading(safe)) < 4 && !isMoving())) { // test issettled
    // std::cout << "in progress" << '\n';
    if (timer.millis().convert(okapi::second) - init > 2) {
      break;
    }
    // std::cout << getHeading() << "\n";
    // std::cout << getHeading(safe) << " " << chassisPidValue << "\n";
    chassisPidValue = chassisTurnPid.step(getHeading(safe));
    chassis->getModel()->tank(chassisPidValue, -1*chassisPidValue);
    rate.delay(100_Hz);
  }
  chassisTurnPid.reset();
  chassis->getModel()->tank(0, 0);
  // allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});

  // std::cout << "turn complete" << '\n';
}

void imuZeroToAngle(double deg, double time){
  chassisTurnPid.setTarget(deg);
  double chassisPidValue;
  okapi::Timer timer;
  double init = timer.millis().convert(okapi::second);
  while (!(abs(deg - getHeading(false)) < 5 && !isMoving())) { // test issettled
    // std::cout << "in progress" << '\n';
    // std::cout << getHeading(false) << "\n";
    if (timer.millis().convert(okapi::second) - init > time) {
      break;
    }
    chassisPidValue = chassisTurnPid.step(getHeading(false));
    chassis->getModel()->tank(chassisPidValue, 0);
    rate.delay(100_Hz);
  }
  chassisTurnPid.reset();
  chassis->getModel()->tank(0, 0);
  chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
}

void odomDriveToPoint(double x, double y, bool forward, double offset, double speedMultiplier, double time, bool persist, bool rush, bool useVision) { // in feet, x is forward, y is sideways
  double copyX = x;
  double copyY = y;
  // turn first
  x -= chassis->getState().x.convert(okapi::foot); // displacement x
  y -= chassis->getState().y.convert(okapi::foot); // displacement y
  double angle = atan(y / x) * 180 / M_PI; // absolute angle
  if (forward) {
    if (x < 0 && y > 0) {
      angle += 180;
    } else if (x < 0 && y < 0) {
      angle -= 180;
    }
  } else {
    if (x > 0 && y > 0) {
      angle -= 180;
    } else if (x > 0 && y < 0) {
      angle += 180;
    }
  }
  imuTurnToAngle(angle);
  jCurve(copyX, copyY, forward, offset, speedMultiplier, time, persist, rush, useVision);
}

void jCurve(double x, double y, bool forward, double offset, double speedMultiplier, double time, bool persist, bool rush, int useVision) { // in feet, x is forward, y is sideways
  // std::cout << vision.get_object_count();

  double copyX = x;
  double copyY = y;
  okapi::MedianFilter<5> visionFilter;

  // pythagorean theorem for distance to travel
  double target = sqrt(powf(copyX-chassis->getState().x.convert(okapi::foot), 2) + powf(copyY-chassis->getState().y.convert(okapi::foot), 2));
  target -= offset;
  if (!forward) {
    target *= -1; // go backwards
  }
  chassisDrivePid.setTarget(target);

  double chassisPidValue; // drive
  double chassisPidValue2; // turn
  double dX, dY; // current displacement
  double encoderReading = 1000000000.0;
  double modified;

  // initial x and y to calculate displacement
  double startX = chassis->getState().x.convert(okapi::foot);
  double startY = chassis->getState().y.convert(okapi::foot);

  okapi::Timer timer;
  double init = timer.millis().convert(okapi::second);

  while (!(abs(target - encoderReading) < 0.25 && !isMoving())) {
    chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
    if (timer.millis().convert(okapi::second) - init > time) {
      if (persist) { // if it should keep going
        okapi::MotorGroup allMotors({kDriveLTPort, kDriveLMPort, kDriveLBPort, kDriveRBPort, kDriveRMPort, kDriveRTPort});
        while (allMotors.getEfficiency() < 50) {
          chassis->getModel()->tank(-1, -1);
        }
      }
      break; // otherwise, just break normally
    }
    if (rush) {
      if (bumper.isPressed() || (abs(target - encoderReading) < 0.25)) {
        // std::cout << bumper.isPressed() << "\n";
        break;
      }
    }
    x = copyX - chassis->getState().x.convert(okapi::foot); // displacement x
    y = copyY - chassis->getState().y.convert(okapi::foot); // displacement y


    if (useVision > 0) {
      if (useVision == 1 && vision1.get_object_count() > 0 && vision1.get_object_count() < 5) {
        chassisPidValue2 = -1 * chassisVisionPid.step(visionFilter.filter(vision1.get_by_size(0).x_middle_coord));
      } else if (useVision == 2 && vision2.get_object_count() > 0  && vision2.get_object_count() < 5){
        chassisPidValue2 = -1 * chassisVisionPid.step(visionFilter.filter(vision2.get_by_size(0).x_middle_coord));
      }
    } else {
      double angle = atan(y / x) * 180 / M_PI; // absolute angle
      if (forward) {
        if (x < 0 && y > 0) {
          angle += 180;
        } else if (x < 0 && y < 0) {
          angle -= 180;
        }
      } else {
        if (x > 0 && y > 0) {
          angle -= 180;
        } else if (x > 0 && y < 0) {
          angle += 180;
        }
      }
      bool safe = angle < -150 || angle > 150;
      if (angle < -150) {
        angle += 360;
      }
      chassisTurnPid.setTarget(angle);

      chassisPidValue2 = chassisTurnPid.step(getHeading(safe));
    }
    dX = chassis->getState().x.convert(okapi::foot) - startX;
    dY = chassis->getState().y.convert(okapi::foot) - startY;
    encoderReading = sqrt(powf(dX, 2) + powf(dY, 2)); // displacement hypotenuse
    if (!forward) {
      encoderReading *= -1;
    }
    // std::cout << encoderReading << " " << target << "\n";

    chassisPidValue = chassisDrivePid.step(encoderReading);
    // chassisPidValue *= speedMultiplier;

    if (rush) {
      if (chassisPidValue > 0) {
        modified = 1;
      } else {
        modified = -1;
      }
    } else {
      if (abs(chassisPidValue) > abs(speedMultiplier)) {
        if (chassisPidValue > 0) {
          modified = speedMultiplier;
        } else {
          modified = speedMultiplier * -1;
        }
      } else {
        modified = chassisPidValue;
      }
    }

    // modified = std::min(chassisPidValue, speedMultiplier);

    // if (rush) {
    //   modified = 1;
    // }

    chassis->getModel()->tank(modified + chassisPidValue2*0.9*abs(modified), modified - chassisPidValue2*0.9*abs(modified));
    // chassis->getModel()->tank(chassisPidValue + chassisPidValue2*0.9, chassisPidValue - chassisPidValue2*0.9);
    rate.delay(100_Hz);
    }

  chassisDrivePid.reset();
  chassisTurnPid.reset();
  chassisVisionPid.reset();
  chassis->getModel()->tank(0, 0);
  chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
  // std::cout << "target: " << copyX << ", " << copyY << "\n";
  // std::cout << "actual: " << chassis->getState().x.convert(okapi::foot) << ", " << chassis->getState().y.convert(okapi::foot) << "\n";
}

void relative(double x, double time) { // in feet, x is forward, y is sideways
  double target = x;
  chassisDrivePid.setTarget(target);

  double chassisPidValue;
  double dX, dY; // current displacement
  double encoderReading = 1000000000.0;

  // initial x and y to calculate displacement
  double startX = chassis->getState().x.convert(okapi::foot);
  double startY = chassis->getState().y.convert(okapi::foot);

  okapi::Timer timer;
  double init = timer.millis().convert(okapi::second);

  while (!(abs(target - encoderReading) < 0.25 && !isMoving())) {
    chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
    dX = chassis->getState().x.convert(okapi::foot) - startX;
    dY = chassis->getState().y.convert(okapi::foot) - startY;
    encoderReading = sqrt(powf(dX, 2) + powf(dY, 2)); // displacement hypotenuse
    if (x < 0) {
      encoderReading *= -1;
    }
    chassisPidValue = chassisDrivePid.step(encoderReading);
    // std::cout << encoderReading << " " << target << "\n";
    chassis->getModel()->tank(chassisPidValue, chassisPidValue);
    if (timer.millis().convert(okapi::second) - init > time) {
      break;
    }

    rate.delay(100_Hz);
  }

  chassisDrivePid.reset();
  chassis->getModel()->tank(0, 0);
}

void climb() {
  double init = imu2.controllerGet();
  okapi::MotorGroup allMotors({kDriveLTPort, kDriveLMPort, kDriveLBPort, kDriveRBPort, kDriveRMPort, kDriveRTPort});
  allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold); // brakemode
  // double target = 10; // target to start incline
  // chassisClimbPid.setTarget(target);
  chassisTurnPid.setTarget(getHeading(false));

  double climbPidValue;
  double turnPidValue;

  chassis->setState({0_ft, 0_ft, 0_deg});

  while (imu2.controllerGet() < init+23) { // pid to start incline on platform; increase this number = stop later, decrease this number = stop earlier
    // climbPidValue = chassisClimbPid.step(imu2.controllerGet());
    turnPidValue = chassisTurnPid.step(getHeading(false)); // keep straight
    // std::cout << encoderReading << " " << target << "\n";
    chassis->getModel()->tank(1 + 0.5 * turnPidValue, 1 - 0.5 * turnPidValue);
    rate.delay(100_Hz);
  }
  // chassisClimbPid.reset();
  // chassisTurnPid.reset();

  chassisClimbPid.setTarget(init);
  // chassisTurnPid.setTarget(getHeading(false));
  // chassis->setMaxVelocity(0);
  std::cout << imu2.controllerGet() << " " << init << "\n";
  while (imu2.controllerGet() - init > 20 || chassis->getState().x.convert(okapi::foot) < 1.5) { // pid for balancing; increase this number = stop earlier, decrease this number = stop later
    std::cout << imu2.controllerGet() << " " << init << "\n";
    climbPidValue = chassisClimbPid.step(imu2.controllerGet());
    turnPidValue = chassisTurnPid.step(getHeading(false));
    // std::cout << encoderReading << " " << target << "\n";
    // std::cout << climbPidValue << "\n";
    // if (climbPidValue > 0 || imu2.controllerGet() - init < 16) {
    //   chassis->getModel()->tank(0.1, 0.1); // platform can balance on its own after u stop moving
    // } else {
      chassis->getModel()->tank(-0.5 * climbPidValue + 0.5 * abs(climbPidValue) * turnPidValue, -0.5 * climbPidValue - 0.5 * abs(climbPidValue) * turnPidValue);
    // }
    // tune these power mulitpliers (might not work with heavy load)
    rate.delay(100_Hz);
  }
  chassis->getModel()->tank(0, 0);
  chassisClimbPid.reset();
  chassisTurnPid.reset();
}

void visionAlign() {
  okapi::MedianFilter<5> visionFilter;
  vision1.set_signature(1, &NEUTRAL);

  chassisVisionPid.setTarget(0);
  double chassisPidValue;

  // std::cout << (abs(vision.get_by_size(0).x_middle_coord) > 5 && isMoving());
  while (abs(vision1.get_by_size(0).x_middle_coord) > 0 || isMoving()) {
    // std::cout << vision.get_by_size(0).x_middle_coord << "\n";
    chassisPidValue = chassisVisionPid.step(visionFilter.filter(vision1.get_by_size(0).x_middle_coord));

    if (vision1.get_object_count() == 0 || abs(chassisPidValue) < 0.08) {
      chassis->getModel()->tank(0, 0);
    } else if (vision1.get_object_count() > 0) {
      chassis->getModel()->tank(-1 * chassisPidValue, chassisPidValue);
    }

    rate.delay(100_Hz);
  }
  chassisVisionPid.reset();
  chassis->getModel()->tank(0, 0);
}
