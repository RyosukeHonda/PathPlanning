#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "vehicle.h"
#include "jmt.h"
#include "helper.h"
#include <vector>

class Trajectory{
public:
    State target_state_s;
    State target_state_d;
    
    std::vector<double> jmt_s;
    std::vector<double> jmt_d;
    
    //Get trajectory according to the behavior and current position of the car
    Trajectory(Vehicle& mycar ,Action action);
    JMT get_jmt_s() const;
    JMT get_jmt_d() const ;
private:
    std::vector<JMT> jmtPair;
    
};

#endif //TRAJECTORY_H_
