#include "vehicle.h"

Vehicle::Vehicle(int id){
  this -> id = id;
  this -> lane = LaneLoc::UNKNOWN;
}

void Vehicle::update_speed(double v){
  this -> v = v;
}

void Vehicle::update_position(double s, double d){
  this -> s = s;
  this -> d = d;
  this -> lane = convert_d_to_lane(d);
}

void Vehicle::get_current_state(State& s, State& d){
  this -> current_state_s = s;
  this -> current_state_d = d;
}

void Vehicle::get_adjacent_lane(){
  if(this -> lane == LaneLoc::LEFT){
    this -> lane_at_left = LaneLoc::NONE;
    this -> lane_at_right = LaneLoc::MIDDLE;
  }else if(this -> lane == LaneLoc::MIDDLE){
    this -> lane_at_left = LaneLoc::LEFT;
    this -> lane_at_right = LaneLoc::RIGHT;
  }else if(this -> lane == LaneLoc::RIGHT){
    this -> lane_at_left = LaneLoc::MIDDLE;
    this -> lane_at_right = LaneLoc::NONE;
  }else{
    this -> lane_at_left = LaneLoc::UNKNOWN;
    this -> lane_at_right = LaneLoc::UNKNOWN;
  }
}

LaneLoc Vehicle::convert_d_to_lane(double d){
  LaneLoc lane = LaneLoc::NONE;
  if( d > 0.0 && d < 4.0){
    lane = LaneLoc::LEFT;
  }else if( d> 4.0 && d < 8.0){
    lane = LaneLoc::MIDDLE;
  }else if( d > 8.0 && d < 12.0){
    lane = LaneLoc::RIGHT;
  }

  return lane;
}

double Vehicle::convert_lane_to_d(LaneLoc lane){
  double d= -1.0; //Initialize the d value(Outside of the lane intentionally)
  if(lane == LaneLoc::LEFT){
    d = 2.2;
  }else if(lane == LaneLoc::MIDDLE){
    d = 6.0;
  }else if(lane == LaneLoc::RIGHT){
    d = 9.8;
  }
  return d;
}

