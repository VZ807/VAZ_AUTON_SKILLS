#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#ifndef roboconfig_h
#define roboconfig_h
	pros::MotorGroup right_mg ({-19, -17, -8},pros::MotorCartridge::green,pros::v5::MotorUnits::rotations);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
    pros::MotorGroup left_mg ({1,5, 18},pros::MotorCartridge::green,pros::v5::MotorUnits::rotations);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
	pros::Motor intake (6,pros::MotorCartridge::blue);
	//pros::Motor lady (1,pros::MotorCartridge::red);
	lemlib::Drivetrain drivetrain (&left_mg,&right_mg,10,lemlib::Omniwheel::OLD_4,435,2);
	pros::Imu imu(9);	
	pros::adi::DigitalOut clamp ('a',LOW);
	pros::Rotation horizontal_encoder(20);
	// vertical tracking wheel encoder
	pros::Rotation vertical_encoder(21);
	// horizontal tracking wheel
	lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
	// vertical tracking wheel
	lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

	lemlib::OdomSensors sensors(nullptr,nullptr,nullptr,nullptr,&imu);

	// lateral PID controller
    lemlib::ControllerSettings lateral_controller(1, //proportional gain (kD)
											  0, //integral gain (kI) 
											  3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
											  );

	// angular PID controller
	lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
											  );

	lemlib::Chassis chassis(drivetrain, // drivetrain settings
							lateral_controller, // lateral PID settings
                        	angular_controller, // angular PID sensors
							sensors
							);
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	double leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	double rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	lemlib::Pose pose(70.471, -21.513, 74.8932);
#endif // _roboconfig_