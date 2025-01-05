#include "main.h"

void subsystem::colorsort () {
    while (true) {
        pros::c::optical_rgb_s_t rgb = color.get_rgb();
        if ((rgb.red > rgb.blue) && (rgb.red > rgb.green) && (rgb.brightness > 50)) {
            //if color sensor close to bottom 
            //intake.move_velocity(600);
            //pros::delay(50); //acount for dist to the top 
            intake.brake();
            pros::delay(50);
            intake.move_velocity(600);
        } 
    }
} 

void subsystem::fishmech (double tardeg, double exitcon) {
    const double kP = 1;
    const double kI = 1;
    const double kD = 1;
    double error = 0;
    double derivitive;
    double integral = 0;
    double preverror;
    double output;
    lemlib::Timer exit(exitcon);
    while (true) {
        preverror = error;
        error = tardeg-fisheye.get_angle();
        if ((abs(error) < .2) || exit.isDone()) {
            fishmechmotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            break;
        } 
        derivitive = error - preverror;
        integral += error;
        if (integral > 360) {
            integral = 360;
        } else if (integral < -360) {
            integral = -360;
        }
        fishmechmotor.move_velocity((kP * error) + (kI * integral) + (kD * derivitive));
    }
}