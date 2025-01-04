#include "main.h"
#include "roboconfig.hpp"

class odom {
    public: 
    struct pose {
    double globalx;
    double globaly;
    double globalheading;
    };
    
    struct offsets {
        double horozontal_encoder_offset;
        double vertical_encoder_offset1;
        double vertical_encoder_offset2;
    };
    

};