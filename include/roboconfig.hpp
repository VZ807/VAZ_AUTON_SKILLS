#ifndef roboconfig_h
#define roboconfig_h
	#include "api.h"
	#include "lemlib/api.hpp" // IWYU pragma: keep
	#include "MCL.hpp"
	extern pros::MotorGroup right_mg;    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
    extern pros::MotorGroup left_mg ;  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
	extern pros::Motor intake;
	extern pros::Motor conveyer;
	//pros::Motor lady (1,pros::MotorCartridge::red);
	extern lemlib::Drivetrain drivetrain;
	extern pros::Imu imu;	
	extern pros::adi::DigitalOut clamp;
	extern pros::Rotation horizontal_encoder;
	// vertical tracking wheel encoder
	extern pros::Rotation vertical_encoder;
	// horizontal tracking wheel
	extern lemlib::TrackingWheel horizontal_tracking_wheel;
	// vertical tracking wheel
	extern lemlib::TrackingWheel vertical_tracking_wheel;

	extern lemlib::OdomSensors sensors;

	// lateral PID controller
    extern lemlib::ControllerSettings lateral_controller;

	// angular PID controller
	extern lemlib::ControllerSettings angular_controller;

	extern lemlib::Chassis chassis;
	extern pros::Controller master;
	extern lemlib::Pose pose;
	extern lemlib::Pose prevpose;
	extern pros::Mutex odom_mutex;
	extern pros::Optical color;
	extern pros::Rotation fisheye;
	extern pros::Motor fishmechmotor;
	extern pros::Distance eye;
	extern monte::distsensconfig distconfig;
#endif // _roboconfig