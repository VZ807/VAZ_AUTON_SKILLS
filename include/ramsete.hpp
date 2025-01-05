#include "main.h"
#include "roboconfig.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
//TODO 
//make function def in src to redice bloat
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
    std::function<void(double,double)> conditions = [](double,double){});
}; 

/**
 * a function to determine the   
 * 
 * 
 */
