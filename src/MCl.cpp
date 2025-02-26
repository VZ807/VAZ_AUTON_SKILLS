#include "MCL.hpp"
#include "roboconfig.hpp"

void monte::carlo () {
    point p1 (-140.88,140.88);
    point p2 (140.88,140.88);
    point p3 (-140.88,-140.88);
    point p4 (140.88,-140.88);
    Rectangle arena (p1,p2,p3,p4); //sturct to create four wall structs
    std::vector<wall> lines = {arena.top,arena.left,arena.right,arena.bottom};
    float dist = eye.get_distance()/25.4;
    double xoff = pose.x - prevpose.x;
    double yoff = pose.y - prevpose.y;
    double noise = 5;
    //weighting vars
    double totalweights = 0;
    double highestweight = 0;
    int highestweightindex = 0;
    //update particles
    for (int i = 0; i < particles.size();++i) {
        //updates all particle positions and adds noise
        particles[i][0] = (particles[i][0] >= map[0] && particles[i][0] <= map[1]) ? (particles[i][0] + randomDoubleInRange(xoff-noise,xoff+noise)) : (particles[i][0] > 0 ? map[1] : map[0]);
        particles[i][1] = (particles[i][1] >= map[0] && particles[i][1] <= map[1]) ? (particles[i][1] + randomDoubleInRange(yoff-noise,yoff+noise)) : (particles[i][1] > 0 ? map[1] : map[0]);
        particles[i][2] = randomDoubleInRange(pose.theta-3,pose.theta+3); 
        particles[i][4] = linerayintersection(particles[i][0],particles[i][1],particles[i][2],lines);
        //weigh particles 
        double sigma = std::max(5.0, dist * 0.1); //CANNOT BE ZERO
        particles[i][3] = exp(-0.5 * pow((particles[i][4]-dist)/sigma,2));
        totalweights += particles[i][3];
        if (highestweight < particles[i][3]) {
            highestweight = particles[i][3];
            highestweightindex = i;
        }
    } 
    //update pose
    chassis.setPose(particles[highestweightindex][0],particles[highestweightindex][1],particles[highestweightindex][2]);

    //normalize and resample particles 
    std::vector<std::vector<float>> resampled;    
    if (totalweights > 0) {
        // Normalize weights, making the sum of all weights 1
        for (int i = 0; i < particles.size(); ++i) {
            particles[i][3] /= totalweights; // Normalize each particle's weight
        }

        //use low variance resampling, a effecient resampling algorithm for computaional speed
        double r = randomDoubleInRange(0.0, 1.0 / particles.size());
        double c = particles[0][3];  
        int v = 0;
        
        for (int j = 0; j < particles.size(); ++j) {
            //chose step values 
            double U = r + j * (1.0 / particles.size());
            //choose particels to be added, particels with higher weightsd are more likely to be selected
            while (U > c) {
                v++;
                c += particles[v][3];
            }
            //add selected particle to resampled array
            resampled.push_back(particles[v]);
        }
        particles = resampled;

        // Reset weights to 1/N after resampling
        double equal_weight = 1.0 / particles.size();
        for (int i = 0; i < particles.size(); ++i) {
            particles[i][3] = equal_weight; // Reset to uniform weight
        } 
         

    }


    
} 

void monte::carloinit (int particlecount) { 
    //crate an array of random particles that are genrally located around the starting position 
    particles.resize(particlecount, std::vector<float>(5, 0));
    for (int i = 0; i < particlecount;++i) {
        particles[i][0] = randomDoubleInRange(pose.x-5,pose.x+5); // x value
        particles[i][1] = randomDoubleInRange(pose.y-5,pose.y+5); // y value 
        particles[i][2] = randomDoubleInRange(pose.theta-5,pose.theta+5); //theta 
        particles[i][3] = 0; //weight 
        particles[i][4] = 0; //simulated distance sensor 
    }  
}

//a raycasting function
double monte::linerayintersection (double x,double y,double theta, std::vector<wall> lines) { 
    theta *= (M_PI/180);
    x = x + distconfig.distsensoffx * cos(theta) - distconfig.distsensoffy * sin(theta);
    y = y + distconfig.distsensoffx * cos(theta) + distconfig.distsensoffy * sin(theta);
    
    //convert to radians
    theta += distconfig.angle;
    if (theta >= M_PI) {
        theta -= 2 * M_PI;  // If the angle exceeds π, subtract 2π
    } else if (theta < -M_PI) {
        theta += 2 * M_PI;  // If the angle is less than -π, add 2π
    }

    //use cramers rule to solve for intersection and distance
    for (int i =0; i < lines.size(); ++i) {
    //calculate denominator for solving the system of parametric equations for t and s values 
    double denom  = (lines[i].p2.x - lines[i].p1.x) * sin(theta) - (lines[i].p2.y - lines[i].p1.y) * cos(theta);

    //if denom is zero or very close to it the lines are almost or exacly parallel meaning we skip this 
    if (std::abs(denom) < 1e-6) {
        continue;
    }
    //distance of the ray being cast  
    double t = ((lines[i].p1.x - x) * (lines[i].p2.y - lines[i].p1.y) - (lines[i].p1.y - y)*(lines[i].p2.x - lines[i].p1.x)) / denom;
    //bound of the segment, used to make sure that the intersection occurs between the two points only 
    double s = ((lines[i].p1.x - x) * sin(theta) - (lines[i].p1.y - y) * cos(theta)) / denom;
    
    //return if intersection is on the bounded line and is pointed away from the ray's origin 
    if (t >= 0 && (0<=s && s<=1)) {
        return t;
    } 
    }
    return -9999;
}

//helper function, does exacply what you think it does 
double monte::randomDoubleInRange(double min, double max) {
    return min + (std::rand() / (RAND_MAX + 1.0)) * (max - min);
} 

