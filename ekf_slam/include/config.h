#ifndef CONFIG_H_
#define CONFIG_H_
#include <eigen3/Eigen/Dense>
using namespace Eigen;

const double PI=3.14159;
const double VEL=3.0;
const double MAX_W=100.0*PI/180.0;  //radians/s
const double CONTROL_DURATION_TIME=0.025;
const double OBSERVE_DURATION_TIME=CONTROL_DURATION_TIME*8;
const double SIGMA_V=0.3;  // m/s
const double SIGMA_W=(3.0*PI/180.0);  //  radians/s
const double SIGMA_R=0.1; //m
const double SIGMA_B=(1.0*PI/180.0);  //radians
const double AT_WAYPOINT=1.0;
const double MAX_RANGE=3.0;  //metres
const double MIN_RADIANS = 60.0*PI/180.0;
const double INF=99999999;

const double GATE_REJECT= 4.0; // maximum distance for association
const double GATE_AUGMENT= 15.0;// minimum distance for creation of new feature
const double GATE_RAD=20.0*PI/180.0;
double NormalizedAngle(double rAngle);




#endif