#include "trajectory.h"
#include <iostream>
using namespace std;

Trajectory::Trajectory(Vehicle& mycar ,Action action){
  //Get current position and velocity
  double target_s = mycar.current_state_s.p + mycar.current_state_s.v * TRAVERSE_TIME;
  double target_v = mycar.current_state_s.v;
  double target_d = mycar.convert_lane_to_d(mycar.lane);


  if(action == Action::KEEP_LANE){
    if(mycar.front_gap > FRONT_GAP_BUFFER){
      target_v = SPEED_LIMIT;
    // This is for the initial step.
    // The first sensor fusion for the front car's gap sometimes 0(Actually it isn't).
    }else if(mycar.front_gap<0.01){
      target_v = SPEED_LIMIT;
    //The car is too near to the front car(Slow Down)
    }else{
      //cout<<"NOT SAFE"<<endl;
      target_v = mycar.front_v - SPEED_BUFFER;
    }
    target_s =  mycar.current_state_s.p + TRAVERSE_TIME * 0.5 * (mycar.current_state_s.v + target_v);
  }


  //Change lane
  if(action == Action::TURN_LEFT){
    target_d = mycar.convert_lane_to_d(mycar.lane_at_left);
  }else if(action == Action::TURN_RIGHT){
    target_d = mycar.convert_lane_to_d(mycar.lane_at_right);
  }

  //Target State for s and d
  this -> target_state_s = {target_s,target_v,0.0};
  this -> target_state_d = {target_d, 0.0, 0.0};

  //Generate Jerk Minimizing Trajectory.
  JMT jmt_s(mycar.current_state_s, target_state_s, TRAVERSE_TIME);
  JMT jmt_d(mycar.current_state_d, target_state_d, TRAVERSE_TIME);

  this -> jmtPair.push_back(jmt_s);
  this -> jmtPair.push_back(jmt_d);
}

JMT Trajectory::get_jmt_s() const {
  return jmtPair[0];
}

JMT Trajectory::get_jmt_d() const {
  return jmtPair[1];
}
