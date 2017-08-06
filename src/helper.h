#ifndef HELPER_H_
#define HELPER_H_

#include <vector>

//const double SPEED_LIMIT = 22.352; // 50mph in m/s
const double SPEED_LIMIT = 20.6; //
const double MIN_SPEED = 20.0;
const double MIN_LANE_CHANGE_SPEED = 13.0;

const double MAX_OBSERVABLE_DIST = 200.0;

const double SAFE_GAP = 100.0;
const double FRONT_GAP_BUFFER = 35.0;
const double SAFE_BEHIND_BUFFER = 15.0;
const double SPEED_BUFFER = 0.7;
const double TRAVERSE_TIME = 1.8;
const double TIME_INCREMENT = 0.02;
const int NUMBER_OF_POINTS = int(TRAVERSE_TIME / TIME_INCREMENT);

const double W_LC = 0;
const double W_DC = 3.0;//3.0;
const double W_VEL = 0;//1.5;
const double W_GAP = 5.6;

const int  PATH_SIZE = 60;
const double TRACK_DISTANCE = 6945.554;


struct State{
    double p;
    double v;
    double a;
};

enum LaneLoc{
    LEFT,MIDDLE,RIGHT,NONE,UNKNOWN
};

enum Action{
    KEEP_LANE,TURN_LEFT,TURN_RIGHT
};

struct XYPoints {
    std::vector<double> xs;
    std::vector<double> ys;
    int n;
};

#endif // HELPER_H_
