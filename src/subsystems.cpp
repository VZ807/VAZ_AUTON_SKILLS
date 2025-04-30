#include "subsystems.hpp"
#include "main.h"
#include "roboconfig.hpp"
#include <cstdio>

void subsystem::colorsort() {
  while (true) {
    pros::c::optical_rgb_s_t rgb = color.get_rgb();
    if ((rgb.red > rgb.blue) && (rgb.red > rgb.green) &&
        (rgb.brightness > 50)) {
      // if color sensor close to bottom
      // intake.move_velocity(600);
      // pros::delay(50); //acount for dist to the top
      intake.brake();
      pros::delay(50);
      intake.move_velocity(600);
    }
  }
}

void subsystem::ladybrown(double tardeg, double exitcon) {
  const double kP = 1;
  const double kI = 0;
  const double kD = 0;
  double error = 0;
  double derivitive;
  double integral = 0;
  double preverror;
  lemlib::Timer exit(exitcon);
  while (true) {
    preverror = error;
    error = tardeg - lady.get_position();
    printf("POS: %f \n", lady.get_position());
    printf("ERROR: %f \n", error);
    if ((fabs(error) < .5) || exit.isDone()) {
      printf("done!");
      lady.move_voltage(0);
      lady.set_brake_mode(MOTOR_BRAKE_HOLD);
      break;
    }
    derivitive = error - preverror;
    integral += error;
    if (integral > 360) {
      integral = 360;
    } else if (integral < -360) {
      integral = -360;
    }
    lady.move_velocity((kP * error) + (kI * integral) + (kD * derivitive));
  }
}
