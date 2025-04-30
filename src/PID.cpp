#include "PID.hpp"
#include "lemlib/timer.hpp"
#include "main.h"
#include <limits>

void PID::turn(double target, double timeout, double cap, double hkP,
               double hkI, double hkD) {
  double heading = imu.get_heading();
  bool PIDT = true;
  double herror = 0;
  double hpreverror;
  double hderivitive;
  double hterror;
  double hvelo;
  lemlib::Timer time(timeout);
  while (PIDT) {
    printf("Target %f\n", target);
    heading = imu.get_heading();
    printf("Heading: %f\n", imu.get_heading());
    herror = heading - target;
    printf("RAWERROR: %f\n", herror);
    if (herror > 180) {
      herror -= 360;
    } else if (herror < -180) {
      herror += 360;
    }

    printf("Error %f\n", herror);
    hterror += herror;
    hderivitive = herror - hpreverror;
    hvelo = std::clamp((hkP * herror) + (hkI * hterror) + (hkD * hderivitive),
                       -cap, cap);
    if (fabs(herror) < 1 || time.isDone()) {
      left_mg.move_velocity(0);
      right_mg.move_velocity(0);
      pros::delay(50);
      herror = imu.get_heading() - target;
      if (herror > 180) {
        herror -= 360;
      } else if (herror < -180) {
        herror += 360;
      }
      PIDT = false;
      pros::delay(20);
      printf("done!");
      return;
    }
    left_mg.move_velocity(-hvelo);
    right_mg.move_velocity(hvelo);

    hpreverror = herror;

    if (hterror > 2000) {
      hterror = 2000;
    }
    pros::Task::delay(20);
  }
}

void PID::drive(double target, double timeout, double cap, double hkP,
                double hkI, double hkD) {
  printf("DRIVE PID STARTED");
  target = (36.0 / 60.0) * (target / (M_PI * 3.25));
  left_mg.tare_position_all();
  right_mg.tare_position_all();
  std::vector<double> leftpos = left_mg.get_position_all();
  std::vector<double> rigpos = right_mg.get_position_all();
  double dist = 0;
  bool PIDT = true;
  double herror = 0;
  double hpreverror = 0;
  double hderivitive = 0;
  double hterror = 0;
  double hvelo = 0;
  lemlib::Timer time(timeout);
  while (PIDT) {
    dist = 0;
    int div = 0;
    printf("Target %f\n", target);
    leftpos = left_mg.get_position_all();
    rigpos = right_mg.get_position_all();
    for (int i = 0; i < leftpos.size() - 1; ++i) {
      if (leftpos[i] != std::numeric_limits<double>::infinity()) {
        dist += leftpos[i];
        ++div;
      }
      if (rigpos[i] != std::numeric_limits<double>::infinity()) {
        dist += rigpos[i];
        ++div;
      }
    }
    dist /= div;
    printf("Dist: %f\n", dist);
    herror = dist - target;
    printf("Error %f\n", herror);
    hterror += herror;
    hderivitive = herror - hpreverror;
    hvelo = std::clamp(((hkP * herror) + (hkI * hterror) + (hkD * hderivitive)),
                       -cap, cap);
    printf("Velo: %f\n", -hvelo);
    if (fabs(herror) < 0.004 || time.isDone()) {
      left_mg.brake();
      right_mg.brake();
      PIDT = false;
      printf("done!");
      pros::delay(20);
      return;
    }
    left_mg.move_velocity(-hvelo);
    right_mg.move_velocity(-hvelo);

    hpreverror = herror;

    if (hterror > 2000) {
      hterror = 2000;
    }
    pros::Task::delay(20);
  }
}
