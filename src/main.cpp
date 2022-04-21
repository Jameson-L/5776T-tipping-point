#include "main.h"

#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/highLift.hpp"
#include "subsystems/powershare.hpp"
#include "autonomous/autonomous.hpp"
#include "autonomous/odometry.hpp"

#define kPneumaticClampPort 7
#define kPneumaticTilterPort 6
#define kPneumaticTransmissionPort 3
// units
// using namespace okapi::literals;

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// okapi::Logger::setDefaultLogger(
  //   std::make_shared<okapi::Logger>(
  //       okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
  //       "/ser/sout", // Output to the PROS terminal
  //       okapi::Logger::LogLevel::debug // Show errors and warnings
  //   )
	// );

	// creating logger
	// auto logger = okapi::Logger::getDefaultLogger();

	// default initialization example
	okapi::Rate rate;
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Among.");
	pros::c::ext_adi_pin_mode(2, kPneumaticClampPort, OUTPUT);
	pros::c::ext_adi_pin_mode(2, kPneumaticTilterPort, OUTPUT);
	pros::c::adi_pin_mode(kPneumaticTransmissionPort, OUTPUT);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::c::adi_digital_write(kPneumaticTransmissionPort, LOW);
	pros::c::ext_adi_digital_write(2, kPneumaticClampPort, HIGH);
	pros::c::ext_adi_digital_write(2, kPneumaticTilterPort, HIGH);

	// while (imu1.isCalibrating() || imu1.isCalibrating()) {
	// 	pros::lcd::set_text(2, "Calibrating IMUs...");
	// 	rate.delay(100_Hz);
	// }
	// pros::lcd::set_text(2, "IMUs done calibrating.");

/*
	okapi::Motor lf = okapi::Motor(5);
	okapi::Motor lb = okapi::Motor(16);
	okapi::Motor rf = okapi::Motor(3);
	okapi::Motor rb = okapi::Motor(13);
	pros::lcd::set_text(3, "LF: " + std::to_string(lf.getTemperature()));
	pros::lcd::set_text(4, "LB: " + std::to_string(lb.getTemperature()));
	pros::lcd::set_text(5, "RF: " + std::to_string(rf.getTemperature()));
	pros::lcd::set_text(6, "RB: " + std::to_string(rb.getTemperature()));
*/
	// initialize IMUs
	// imu1.calibrate();
	// imu2.calibrate();

	// completion notification
	// std::cout << "IMU initialization complete." << '\n';
	// pros::lcd::set_text(1, "IMU initialization complete.");

	// messages
	// LOG_DEBUG_S("Initializing...");
	// LOG_DEBUG_S("Initialization Complete.");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	okapi::MotorGroup allMotors({kDriveLTPort, kDriveLMPort, kDriveLBPort, kDriveRBPort, kDriveRMPort, kDriveRTPort});
	allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	pros::c::ext_adi_pin_mode(2, kPneumaticClampPort, OUTPUT);
	pros::c::ext_adi_pin_mode(2, kPneumaticTilterPort, OUTPUT);
	pros::c::adi_digital_write(kPneumaticTransmissionPort, LOW);
	pros::c::ext_adi_digital_write(2, kPneumaticClampPort, HIGH);
	pros::c::ext_adi_digital_write(2, kPneumaticTilterPort, HIGH);
	// right();
	// rightOne();
	// rightAllianceWP();
	// left();
	// leftOne();
	// leftCounter();
	// soloAWP();
	//
	// skills();
	vision1.set_signature(1, &NEUTRAL);
	jCurve(4, 0, true, 0, 1, 5, false, false, 1);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

double accelerationLimiter(double targetThrottle, double lastThrottle, double gain){
	if(lastThrottle <= targetThrottle){
		return std::min(lastThrottle + gain, targetThrottle);
	} else {
		return std::max(lastThrottle - gain, targetThrottle);
	}
}

void opcontrol() {
	// OUR CODE:
	// creating logger
	auto logger = okapi::Logger::getDefaultLogger();
	okapi::Controller controller;
	okapi::Controller controller2 = okapi::Controller(okapi::ControllerId::partner);
	okapi::Timer timer;

	okapi::MedianFilter<10> highLiftFilter;
	okapi::MedianFilter<10> lowLiftFilter;

	// power variables
	double leftY;
	double rightY;

	double highLiftPidValue;

	// bool reverseDrive = false;
	bool holdDrive = false;
	bool lowLiftToggle = false;
	bool highLiftOff = false;
	int highLiftToggle = 0;
	int powershareToggle = 3;
	bool clampToggle = false;
	bool reset = true; // make sure u let go of both l1 and l2 after a double tap before trying to do either l1 or l2

	okapi::MotorGroup allMotors({kDriveLTPort, kDriveLMPort, kDriveLBPort, kDriveRBPort, kDriveRMPort, kDriveRTPort});

	okapi::Rate rate;
	if (controller[okapi::ControllerDigital::R1].isPressed()) {
		pros::c::ext_adi_digital_write(2, kPneumaticClampPort, HIGH);
		highLiftToggle = 3;
	} else {
		pros::c::ext_adi_digital_write(2, kPneumaticClampPort, LOW);
	}
	if (controller[okapi::ControllerDigital::R2].isPressed()) {
		pros::c::ext_adi_digital_write(2, kPneumaticTilterPort, HIGH);
	} else {
		pros::c::ext_adi_digital_write(2, kPneumaticTilterPort, LOW);
	}
	pros::c::adi_digital_write(kPneumaticTransmissionPort, LOW);

	// to stop auton tasks
	continueHighLift = false;
	continueLowLift = false;

	// rate.delay(40_Hz);

	// vision.set_signature(1, &NEUTRAL);
	powershare.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	while (true) {
		// std::cout <<"running op control" << "\n";

		// printing odometry tests
		// okapi::OdomState pos = chassis->getState();
		// std::cout << "left: " << LTrackingWheel.controllerGet() << '\n';
		// std::cout << "right: " << RTrackingWheel.controllerGet() << '\n';
		// std::cout << "middle: " << MTrackingWheel.controllerGet() << '\n';
		// std::cout << "x-pos: " << pos.x.convert(okapi::foot) << '\n';
		// std::cout << "y-pos: " << pos.y.convert(okapi::foot) << '\n';
		// std::cout << "theta: " << pos.theta.convert(okapi::degree) << ' ';
		// std::cout<< "imu: " << getHeading() << '\n';
		// std::cout << bumper.isPressed() << "\n";
		// std::cout << allMotors.getEfficiency() << "\n";

		// if (bumper.changedToPressed()) {
			// std::cout << imu2.controllerGet() << "\n";
			// std::cout << vision.get_object_count() << "\n";
			// std::cout << vision.get_by_size(0).x_middle_coord << " " << vision.get_by_size(0).y_middle_coord << "\n";
		// }

		// std::cout << "lowLiftPot: " << lowLiftPot.controllerGet() << '\n';
		// std::cout << "highLiftPot: " << highLiftFilter.filter(highLiftLPot.controllerGet())  << '\n';

		// std::cout << "x: " << vision.get_by_size(0).x_middle_coord << "\n";
		// std::cout << "y: " << vision.get_by_size(0).y_middle_coord << "\n";
		// std::cout << LBDistanceSensor.getObjectSize() << "\n";

		// set power variables
		leftY = controller.getAnalog(okapi::ControllerAnalog::leftY);
		rightY = controller.getAnalog(okapi::ControllerAnalog::rightY);


		// toggles
		if(controller[okapi::ControllerDigital::A].changedToPressed()) {
			highLiftOff = !highLiftOff;
		}

		if (controller.getAnalog(okapi::ControllerAnalog::leftX) == -1 && controller.getAnalog(okapi::ControllerAnalog::rightX) == 1) {
			holdDrive = false;
			controller.setText(0, 0, "coast");
		}
		if (controller.getAnalog(okapi::ControllerAnalog::leftX) == 1 && controller.getAnalog(okapi::ControllerAnalog::rightX) == -1) {
			holdDrive = true;
			controller.setText(0, 0, "hold ");
		}

		if(controller[okapi::ControllerDigital::X].changedToPressed()) {
			if (powershareToggle == 1) {
				powershareToggle = 0;
			} else {
				powershareToggle = 1;
			}
		}

		if(controller[okapi::ControllerDigital::Y].changedToPressed()) {
			if (powershareToggle == 2) {
				powershareToggle = 0;
			} else {
				powershareToggle = 2;
			}
		}

		if(controller[okapi::ControllerDigital::B].changedToPressed()) {
			if (powershareToggle == 3) {
				powershareToggle = 0;
			} else {
				powershareToggle = 3;
			}
		}

		if(controller[okapi::ControllerDigital::down].changedToPressed()) {
			highLiftToggle = 0;
		}

		if(controller[okapi::ControllerDigital::R1].changedToPressed()) {
			clampToggle = !clampToggle;
				if (clampToggle) {
					pros::c::ext_adi_digital_write(2, kPneumaticClampPort, HIGH);
					if (highLiftToggle == 0) {
						highLiftToggle = 3;
					}
				} else {
					pros::c::ext_adi_digital_write(2, kPneumaticClampPort, LOW);
					if (highLiftToggle == 3) {
						highLiftToggle = 0;
					}
				}
		}

		if(controller[okapi::ControllerDigital::R2].changedToPressed()) {
				lowLiftToggle = !lowLiftToggle;
				if (lowLiftToggle) {
					pros::c::ext_adi_digital_write(2, kPneumaticTilterPort, LOW);
				} else {
					pros::c::ext_adi_digital_write(2, kPneumaticTilterPort, HIGH);
				}
		}

		if (controller[okapi::ControllerDigital::L1].isPressed() && controller[okapi::ControllerDigital::L2].isPressed()) {
			highLiftOff = false;
			highLiftToggle = 1;
			reset = false;
		} else if(controller[okapi::ControllerDigital::L1].changedToPressed()) {
			highLiftOff = false;
			if (reset) {
				highLiftToggle = 2;
			}
		} else if(controller[okapi::ControllerDigital::L2].changedToPressed()) {
			highLiftOff = false;
			if (reset) {
				if (clampToggle) {
					highLiftToggle = 3;
				} else {
					highLiftToggle = 0;
				}
			}
		} else {
			reset = true;
		}

		// pid
		if(highLiftToggle == 2) {
			// up
			highLiftPid.setTarget(kHighLiftUpTarget);
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.15 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		} else if (highLiftToggle == 0) {
			// down
			highLiftPid.setTarget(kHighLiftDownTarget);
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.1 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		} else if (highLiftToggle == 3) {
			// kinda down
			highLiftPid.setTarget(kHighLiftHoldTarget);
			if (highLiftLPot.controllerGet() <= kHighLiftHoldTarget - 100) {
				highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.09 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
				highLiftPidValue *= 3.5;
			} else {
				highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.09 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
			}
		} else {
			// placing height
			highLiftPid.setTarget(kHighLiftMidTarget);
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.15 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		}

		if (powershareToggle == 1) {
			// powershare.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			powersharePid.setTarget(powershareTarget);
			powershare.controllerSet(powersharePid.step(lowLiftFilter.filter(powersharePot.controllerGet())));
		} else if (powershareToggle == 2){
			// powershare.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			powershare.controllerSet(-1);
		} else if (powershareToggle == 3){
			// powershare.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			if (powersharePot.controllerGet() > powershareTarget2+50) {
				powershare.controllerSet(-1);
			} else {
				powershare.controllerSet(0);
			}
		} else {
			// powershare.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			powershare.controllerSet(0);
		}

		chassis->getModel()->tank(leftY, rightY);

		if (holdDrive) {
			pros::c::adi_digital_write(kPneumaticTransmissionPort, HIGH);
			allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		} else {
			pros::c::adi_digital_write(kPneumaticTransmissionPort, LOW);
			allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
		}

		// highLiftOff = true;
		if (highLiftOff) {
			highLift.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			highLift.controllerSet(0);
		} else {
			highLift.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			highLift.controllerSet(highLiftPidValue);
		}

		rate.delay(100_Hz);
	}
}
