#include "main.h"
#ifndef roboconfig_h
#define roboconfig_h
	pros::MotorGroup right_mg ({-19, -17, -8},pros::MotorCartridge::green,pros::v5::MotorUnits::rotations);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
    pros::MotorGroup left_mg ({1,5, 18},pros::MotorCartridge::green,pros::v5::MotorUnits::rotations);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
	pros::Motor intake (6,pros::MotorCartridge::blue);
	//pros::Motor lady (1,pros::MotorCartridge::red);
	pros::Imu imu(9);	
	pros::adi::DigitalOut clamp ('a',LOW);
	pros::Rotation horizontal_encoder(20);
	// vertical tracking wheel encoder
	pros::Rotation vertical_encoder(21);
	// horizontal tracking wheel
	pros::Controller master(pros::E_CONTROLLER_MASTER);
#endif // _roboconfig_