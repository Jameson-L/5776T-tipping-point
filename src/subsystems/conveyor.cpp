#include "subsystems/conveyor.hpp"

const int8_t kConveyorPort = 1; // -6

okapi::Motor conveyorMotor = okapi::Motor(kConveyorPort);

okapi::IterativeVelPIDController conveyorVelPID = okapi::IterativeControllerFactory::velPID(0.0001, 0.0, 0.0);
