#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <vector>
#include <string>
#include <cmath>
#include "helper.h"
#include "vehicle.h"


class BehaviorPlanner{
public:
    BehaviorPlanner();
    double front_v;
    double front_s;
    double front_gap;
    double adjacent_front_v;
    double adjacent_front_s;
    double adjacent_front_left_gap;
    double adjacent_front_right_gap;
    double adjacent_behind_left_gap;
    double adjacent_behind_right_gap;
    
    double get_gap(Vehicle& mycar, std::vector<Vehicle>& otherCars);
    double get_gap_adjacent_front(Vehicle& mycar, std::vector<Vehicle>& otherCars, std::string direction);
    double get_gap_adjacent_behind(Vehicle& mycar, std::vector<Vehicle>& otherCars, std::string direction);
    double lane_change_cost(Vehicle& mycar);
    double distance_cost(Vehicle& mycar, std::string direction);
    double velocity_cost(Vehicle& mycar);
    double gap_cost(Vehicle& mycar, std::string direction);
    double calc_cost(Vehicle& mycar, std::vector<Vehicle>& otherCars, std::string direction);
    Action get_action(Vehicle& mycar, std::vector<Vehicle>& otherCars);
};


#endif // VEHICLE_H_
