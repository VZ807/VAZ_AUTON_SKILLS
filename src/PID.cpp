#include "PID.hpp"
#include "main.h"
#include<limits>
#include "lemlib/timer.hpp"


void PID::turn(double target,double hkP, double hkI,double hkD) {
		double heading = imu.get_heading();
		bool PIDT = true;
		double herror = 0;
		double hpreverror;
		double hderivitive;
		double hterror;
		double hvelo;
		while (PIDT) {
		printf("Target %f\n",target);
		heading = imu.get_heading();
		printf("Heading: %f\n", imu.get_heading());
		herror = heading - target;
		if (herror > 180) {
                herror -= 360;  
            } else if (herror < -180) {
                herror += 360;  
            }

		printf("Error %f\n", herror);
		hterror += herror;
		hderivitive = herror - hpreverror;
		hvelo = (hkP*herror)+(hkI*hterror)+(hkD*hderivitive);
		if (fabs(herror) < 0.7)  {
			left_mg.move_velocity(0);
			right_mg.move_velocity(0);
			PIDT = false;
			left_mg.move_velocity(0);
			right_mg.move_velocity(0);
			pros::delay(20);
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

	void PID::drive(double target, double timeout, double cap,double hkP, double hkI,double hkD) {
		printf("DRIVE PID STARTED");
		target = (target/11.78097245) * 2;
		left_mg.tare_position();
		right_mg.tare_position();
		std::vector<double> leftpos = left_mg.get_position_all();
		std::vector<double> rigpos = right_mg.get_position_all();
		double dist = 0;
		bool PIDT = true;
		double herror = 0;
		double hpreverror;
		double hderivitive = 0;
		double hterror;
		double hvelo;
		lemlib::Timer time(timeout);
		while (PIDT) {
		dist = 0;
		printf("Target %f\n",target);
		leftpos = left_mg.get_position_all();
		rigpos = right_mg.get_position_all();
		for (int i = 0; i < leftpos.size()-1;++i) {
			if (leftpos[i] != std::numeric_limits<double>::infinity()) {
				dist += leftpos[i];
			}
			if (rigpos[i] != std::numeric_limits<double>::infinity()) {
				dist += rigpos[i];
			}
		}
		dist /= 6;
		printf("Dist: %f\n", dist);
		herror = dist - target;
		printf("Error %f\n", herror);
		hterror += herror;
		hderivitive = herror - hpreverror;
		hvelo = std::clamp(((hkP*herror)+(hkI*hterror)+(hkD*hderivitive)),-cap,cap);
		printf("Velo: %f\n",-hvelo);
		if (fabs(herror) < .1 || time.isDone())  {
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