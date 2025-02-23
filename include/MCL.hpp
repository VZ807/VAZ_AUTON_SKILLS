#ifndef MCL_H
#define MCL_H 
#include<vector>
class monte {  
    public:
    //struct to hold particle data
    std::vector<std::vector<float>> particles;
    
    std::vector<double> map = {-140.88,140.88}; 
    
    struct point {
    double x, y;
    point(double x = 0, double y = 0) : x(x), y(y) {}
}; 

struct wall {
    point p1;
    point p2;
    wall(point p1, point p2) : p1(p1), p2(p2) {}
};

struct Rectangle {
    point TL; 
    point TR;
    point BL;
    point BR;
    wall top; 
    wall bottom;
    wall left;
    wall right;

    Rectangle(point TL, point TR, point BL, point BR) 
        : TL(TL), TR(TR), BL(BL), BR(BR), 
          top(TL, TR), bottom(BL, BR), left(TL, BL), right(TR, BR) {} 
};

    void carloinit (int particlecount);
    std::vector<double> carlo();
    double randomDoubleInRange(double min,double max);
    double linerayintersection (double x,double y,double theta, std::vector<wall> lines); 

    struct distsensconfig {
    float distsensoffx;
    float distsensoffy;
    float angle;
    distsensconfig(float distsensoffx,float distsensoffy,float angle) : 
        distsensoffx(distsensoffx), distsensoffy(distsensoffy), angle(angle) {}
    };



}; 
#endif //MCL_H