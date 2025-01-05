#include "main.h"
#include "roboconfig.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
class ramsete {
    public:
    /**
     * Implementation for the ramsete algoritm 
     * 
     * \param path  
     *      input points for path here in {x,y,theta,linvel,angvel,direction} 
     * \param linvel
     *      provide linear velocity to get to the starting point 
     * \param angvel 
     *      provide linear velocity to get to the starting point 
     * \param finalheading 
     *      provide orientation at the end of the path 
     * \param settletime 
     *      provide maximum time to reach point 
     * \param condition 
     *      provide actions to run while on the path
     */
    void follow (std::vector<std::vector<double>> path, double initlinvel, double initangvel, double finalheading, double settletime, 
    std::function<void(double,double)> conditions = [](double,double){return;}) {
        double currentx;
        double currenty;
        double currentheading;
        double targetx;
        double targety;
        double targetheading;
        double targetlinvel;
        double targetangvel;
        double direction;
        bool backwards = false;
        lemlib::Timer time (settletime); 
        //loop over entire vector ecept for last point (due to redundancy in path generation)
        for (int i = 0;i <= path.size()-1; ++i) {
            direction = path[i][5];
            //change the first and last params occording to inputs 
            if (i = 0) {
            targetx = path[i][0];
            targety = path[i][1];
            targetheading = path[i][2];
            targetlinvel = initlinvel;
            targetangvel = initangvel;    
            } else if (i == path.size()) {
            targetx = path[i][0];
            targety = path[i][1];
            targetheading = finalheading;
            targetlinvel = 0;
            targetangvel = 0;
            } else {
            targetx = path[i][0];
            targety = path[i][1];
            targetheading = path[i][2];
            targetlinvel = path[i][3];
            targetangvel = path[i][4];
            }
            //invert heading and reverse ang and lin velocities to move backwards 
            if (direction == 1) {
                backwards != backwards;
                if (backwards == true) {
                if (targetheading > 180 || targetheading == 180) {
                    targetheading -= 180;
                } else {
                    targetheading += 180;
                }
                targetlinvel *= -1;
                targetangvel *= -1; 
                }
            } 
            // keep adjusting until within a tolerance 
            time.set(settletime);
            while (true) {
            //fetch current x,y values from odom functions
            pose = chassis.getPose(); 
            currentx = pose.x;
            currenty = pose.y;
            currentheading = pose.theta;
            //calculate error in the robot's local frame 
            double errorx = (targetx - currentx)*cos(currentheading) + (targety - currenty)*sin(currentheading);
            double errory = -(targety - currenty)*sin(currentheading) + (targetx - currentx)*cos(currentheading);
            double errorheading = targetheading - currentheading;
            //exit within tolerance
            if ((abs(errorx) < .2 && abs(errory) < .2 && abs(errorheading) < .5 )|| time.isDone() == true) {
                break;
            }
            //compute gainvalues 
            double b = 1; 
            double zeta = 1; 
            double k = 2 * zeta * sqrt((pow(targetangvel,2) + b) * pow(targetlinvel,2)); //gainvalue
            //compute output velocities 
            double linvel = targetlinvel * cos(errorheading) + k * errorx;
            double angvel;
            if (errorheading != 0) {
                angvel = targetangvel + k * errorheading + ((b * targetlinvel * sin(errorheading) * errory)/errorheading);
            } else {
                angvel = 0;
            }
            //convert to wheels and output to motors 
            //linvel  = (linvel);
            left_mg.move_velocity(linvel + angvel);
            right_mg.move_velocity(linvel - angvel); 

            pros::delay(10); 
            }
            conditions(currentx,currenty);
        } 
    }

}; 

/**
 * a function to determine the   
 * 
 * 
 */
bool withintol (double var,double check,double tol = .2) {
    if ((check-tol) < var < (check+tol)) {
        return true;
    } else return false;
}