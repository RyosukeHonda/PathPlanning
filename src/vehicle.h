#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <vector>
#include "helper.h"


class Vehicle{
public:
    int id;
    double v;
    double s;
    double d;
    double front_v;
    double front_s;
    double front_gap;
    double adjacent_v;
    double adjacent_gap;    
    State current_state_s;
    State current_state_d;
    
    LaneLoc lane;
    LaneLoc lane_at_left;
    LaneLoc lane_at_right;
    
    Vehicle(int id);
    void update_speed(double v);
    void update_position(double s, double d);
    void get_current_state(State& s, State& d);
    void get_adjacent_lane();
    LaneLoc convert_d_to_lane(double d);
    double convert_lane_to_d(LaneLoc lane);
};


#endif // VEHICLE_H_
