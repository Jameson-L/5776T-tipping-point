#include "main.h"

#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/highLift.hpp"
#include "subsystems/conveyor.hpp"
#include "autonomous/autonomous.hpp"
#include "autonomous/odometry.hpp"

#define kPneumaticClawPort 7
#define kPneumaticLiftPort 2
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
	pros::c::adi_pin_mode(kPneumaticClawPort, OUTPUT);
	pros::c::adi_pin_mode(kPneumaticLiftPort, OUTPUT);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
	pros::c::adi_digital_write(kPneumaticLiftPort, LOW);

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
	pros::c::adi_pin_mode(kPneumaticClawPort, OUTPUT);
	pros::c::adi_pin_mode(kPneumaticLiftPort, OUTPUT);
	pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
	pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
	// right();
	// rightOne();
	// rightAllianceWP();
	// left();
	// leftOne();
	// leftCounter();
	// soloAWP();
	//
	// skills();
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

	// power variables
	double leftY;
	double rightY;

	double lastLeftY = 0;
	double lastRightY = 0;

	double highLiftPidValue;

	// bool reverseDrive = false;
	bool holdDrive = false;
	bool lowLiftToggle = false;
	bool highLiftOff = false;
	int highLiftToggle = 0;
	bool conveyorToggle = false;
	bool conveyorToggle2 = false;
	bool clampToggle = false;
	bool rumble = true;
	bool reset = true; // make sure u let go of both l1 and l2 after a double tap before trying to do either l1 or l2
	bool wasDown = true;

	okapi::MotorGroup allMotors({kDriveLTPort, kDriveLMPort, kDriveLBPort, kDriveRBPort, kDriveRMPort, kDriveRTPort});

	okapi::Rate rate;
	if (controller[okapi::ControllerDigital::R1].isPressed()) {
		pros::c::adi_digital_write(kPneumaticClawPort, LOW);
		highLiftToggle = 3;
	} else {
		pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
	}
	if (controller[okapi::ControllerDigital::R2].isPressed()) {
		pros::c::adi_digital_write(kPneumaticLiftPort, LOW);
	} else {
		pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
	}

	// to stop auton tasks
	continueHighLift = false;

	rate.delay(40_Hz);

	// vision.set_signature(1, &NEUTRAL);

	while (true) {
		// std::cout <<"running op control" << "\n";

		// printing odometry tests
		okapi::OdomState pos = chassis->getState();
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
		// if (leftY != 0 && rightY != 0) {
			// 	taskRunning = false;
		// }


		if (highLiftOff) {
			highLift.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
		} else {
			highLift.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		}

		// toggles
		if(controller[okapi::ControllerDigital::A].changedToPressed()) {
			highLiftOff = !highLiftOff;
		}

		if(controller[okapi::ControllerDigital::B].changedToPressed()) {
			holdDrive = !holdDrive;
			holdDrive ? controller.setText(0, 0, "hold ") : controller.setText(0, 0, "coast");
		}
		if (controller.getAnalog(okapi::ControllerAnalog::leftX) == -1 && controller.getAnalog(okapi::ControllerAnalog::rightX) == 1) {
			holdDrive = false;
			holdDrive ? controller.setText(0, 0, "hold ") : controller.setText(0, 0, "coast");
		}
		if (controller.getAnalog(okapi::ControllerAnalog::leftX) == 1 && controller.getAnalog(okapi::ControllerAnalog::rightX) == -1) {
			holdDrive = true;
			holdDrive ? controller.setText(0, 0, "hold ") : controller.setText(0, 0, "coast");
		}

		if(controller[okapi::ControllerDigital::X].changedToPressed()) {
			if (conveyorToggle2) {
				conveyorToggle2 = false;
				conveyorToggle = false;
			} else {
				conveyorToggle = !conveyorToggle;
			}
		}

		if(controller[okapi::ControllerDigital::Y].changedToPressed()) {
			if (!conveyorToggle) {
				conveyorToggle = true;
				conveyorToggle2 = true;
			} else {
				conveyorToggle2 = !conveyorToggle2;
			}
		}

		if (conveyorToggle) {
			if (highLiftToggle == 0) {
				wasDown = true;
				highLiftToggle = 3;
			}
		} else {
			if (wasDown) {
				highLiftToggle = 0;
				wasDown = false;
			}
		}

		if(controller[okapi::ControllerDigital::up].changedToPressed()) {
			highLiftToggle = 4;
		}
		if(controller[okapi::ControllerDigital::down].changedToPressed()) {
			highLiftToggle = 0;
		}
		/*
		if(controller[okapi::ControllerDigital::left].changedToPressed()) {
			pros::Task placeMogo(place);
		}
		if(controller[okapi::ControllerDigital::right].changedToPressed()) {
			pros::Task tilterToFourbar(tilterToLift);
			taskRunning = true;
			if (!taskRunning) {
				// pros::Task task_suspend(tilterToLift);
				tilterToFourbar.suspend();
				taskRunning = false;
			}
		}
		// */

		if(controller[okapi::ControllerDigital::R1].changedToPressed()) {
			clampToggle = !clampToggle;
				if (clampToggle) {
					pros::c::adi_digital_write(kPneumaticClawPort, LOW);
					if (highLiftToggle == 0) {
						highLiftToggle = 3;
					}
				} else {
					pros::c::adi_digital_write(kPneumaticClawPort, HIGH);
					if (highLiftToggle == 3) {
						highLiftToggle = 0;
					}
				}
		}

		if(controller[okapi::ControllerDigital::R2].changedToPressed()) {
				lowLiftToggle = !lowLiftToggle;
				if (lowLiftToggle) {
					pros::c::adi_digital_write(kPneumaticLiftPort, LOW);
				} else {
					pros::c::adi_digital_write(kPneumaticLiftPort, HIGH);
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
			// std::cout << vision.print_signature(NEUTRAL);
			// std::cout << vision.get_object_count() << "\n";

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

		// partner controller
		if (controller2[okapi::ControllerDigital::L1].changedToPressed() || controller2[okapi::ControllerDigital::R1].changedToPressed()) {
			if (conveyorToggle2) {
				conveyorToggle2 = false;
				conveyorToggle = false;
			} else {
				conveyorToggle = !conveyorToggle;
			}
		}
		if(controller2[okapi::ControllerDigital::L2].changedToPressed() || controller2[okapi::ControllerDigital::R2].changedToPressed()) {
			if (!conveyorToggle) {
				conveyorToggle = true;
				conveyorToggle2 = true;
			} else {
				conveyorToggle2 = !conveyorToggle2;
			}
		}

		// pid
		if(highLiftToggle == 2) {
			// up
			highLiftPid.setTarget(kHighLiftUpTarget);
			// highLiftPidValue = highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.15 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		} else if (highLiftToggle == 0) {
			// down
			highLiftPid.setTarget(kHighLiftDownTarget);
			conveyorToggle = false;
			// highLiftPidValue = highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.1 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		} else if (highLiftToggle == 3) {
			// kinda down
			highLiftPid.setTarget(kHighLiftHoldTarget);
			// highLiftPidValue = highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
			if (highLiftLPot.controllerGet() <= 1000) {
				highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.09 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
				highLiftPidValue *= 3.5;
			} else {
				highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.09 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
			}
		} else if (highLiftToggle == 4) {
			// down
			highLiftPid.setTarget(kHighLiftMaxTarget);
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.1 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		} else {
			// placing height
			highLiftPid.setTarget(kHighLiftMidTarget);
			// highLiftPidValue = highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
			highLiftPidValue = std::abs(highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()))) < 0.15 ? 0 : highLiftPid.step(highLiftFilter.filter(highLiftLPot.controllerGet()));
		}

		// set power
		// if(highLiftToggle == 1 || highLiftToggle == 2){
		// 	leftY = accelerationLimiter(leftY, lastLeftY, 0.01);
		// 	rightY = accelerationLimiter(rightY, lastRightY, 0.01);
		// 	lastLeftY = leftY;
		// 	lastRightY = rightY;
		// }
		// if(!reverseDrive){
		// if (!taskRunning) {
			chassis->getModel()->tank(leftY, rightY);
		// }
		// } else {
		// 	chassis->getModel()->tank(-rightY, -leftY);
		// }
		if (holdDrive) {
			allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		} else {
			allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
		}
		if (conveyorToggle) {
			if (highLiftLPot.controllerGet() > 1000) {
				conveyorMotor.controllerSet(conveyorToggle2 ? -1 : 1);
			}
		} else {
			conveyorMotor.controllerSet(0);
		}
		highLiftOff = true;
		if (highLiftOff) {
			highLift.controllerSet(0);
		} else {
			highLift.controllerSet(highLiftPidValue);
		}

		rate.delay(100_Hz);
	}
}
