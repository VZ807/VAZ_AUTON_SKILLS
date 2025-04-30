#include "main.h"
#include "PID.hpp"
#include "pros/misc.h"
#include "roboconfig.hpp"
#include "robodash/api.h"
#include "robodash/views/image.hpp"
#include "subsystems.hpp"
#include <cstdio>
#include <vector>
extern lv_img_dsc_t mqdefault;
bool withintol(double var, double check, double tol = .2) {
  if ((check - tol) < var < (check + tol)) {
    return true;
  } else
    return false;
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
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
void SWP() {
  PID pid;
  imu.set_heading(315);
  conveyer.move_relative(100, 100);
  pid.drive(2, 2000); 
  pid.turn(135);
  pid.drive(-20,5000);
}

rd::Selector selector({{"Good auton", SWP}});
rd::Image image(&mqdefault, "lebron");
void initialize() {
  // here
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
 * starts.:
 */
void competition_initialize() {}

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
  chassis.calibrate();
    SWP();
  //selector.run_auton();
  // chassis.calibrate();
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
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  subsystem sub;
  bool clampstate = false;
  while (true) {
    pros::lcd::print(0, "%d %d %d",
                     (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                     (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                     (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >>
                         0); // Prints status of the emulated screen LCDs

    // Arcade control scheme
    int dir = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    // std::vector<double> dist = left_mg.get_position_all();
    // printf("Dist: %f \n", (dist[0] + dist[1] + dist[2]) / 3);
    printf("lady deg: %f\n", lady.get_position());
    if (master.get_digital(DIGITAL_R2)) {
      conveyer.move_velocity(450);
      intake.move_velocity(-450);
    } else if (master.get_digital(DIGITAL_R1)) {
      conveyer.move_velocity(-450);
      intake.move_velocity(450);

    } else {
      conveyer.move_velocity(0);
      intake.move_velocity(0);
    }
    if (master.get_digital(DIGITAL_Y)) {
      clampstate = !clampstate;
      clamp.set_value(clampstate);
      pros::delay(500);
    }
    chassis.curvature(dir, turn);
    pros::delay(20); // Run for 20 ms then update
  }
}
