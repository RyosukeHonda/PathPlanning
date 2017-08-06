#include "behavior_planner.h"
#include "vehicle.h"
#include <vector>
#include <string>
#include <cmath>
#include <iostream>


using namespace std;

BehaviorPlanner::BehaviorPlanner(){};

double BehaviorPlanner::get_gap(Vehicle& mycar, vector<Vehicle>& otherCars){
  double gap;
  double min_gap = 100000000;

  if(mycar.lane == LaneLoc::NONE || mycar.lane == LaneLoc::UNKNOWN){
    return 0.000001;
  }
  int count_same_lane = 0;
  int count_behind = 0;
  for(auto car : otherCars){
    //Target car is on the same lane
    if(mycar.lane == car.lane){
      count_same_lane +=1;
      gap = car.s - mycar.s;

      //The number of cars behind.
      if(gap < 0){
        count_behind +=1;
      }
      //Calculate minimum gap
      if(gap > 0 && gap < min_gap){
        min_gap = gap;
        this->front_v = car.v;
        this->front_s = car.s;
        this->front_gap = min_gap;
        mycar.front_gap = min_gap;
      }
    }
  }
  //If there's no car in front of the driver, the distance will be the MAX_OBSERVABLE_DIST
  if(count_same_lane == count_behind){
      this ->front_gap = MAX_OBSERVABLE_DIST;
    }
  return min_gap;
}

double BehaviorPlanner::get_gap_adjacent_front(Vehicle& mycar, vector<Vehicle>& otherCars, string direction){
  double gap_left;
  double gap_right;
  double min_gap = 100000000;

  //For left lane
  if(direction.compare("LEFT") == 0){
    int count_left =0;
    int count_left_behind = 0;
    for(auto car : otherCars){

      if(mycar.lane_at_left == car.lane){
        count_left+=1;
        gap_left = car.s - mycar.s;
        //The number of cars behind left
        if(gap_left<0){
          count_left_behind +=1;
        }
        if( gap_left > 0 && gap_left < min_gap){
          min_gap = gap_left;
          this -> adjacent_front_v = car.v;
          this -> adjacent_front_left_gap = min_gap;
        }
      }
      //Occurs when the car is in the left lane.
      if(mycar.lane_at_left == LaneLoc::NONE){
        this -> adjacent_front_left_gap = 0.0;
      }
    }

    //If all the left car is behind left, we set the front gap is MAX_OBSERVABLE_DIST.
    if(count_left == count_left_behind){
      this -> adjacent_front_left_gap = MAX_OBSERVABLE_DIST;
    }
  //For right lane
  }else if(direction.compare("RIGHT") == 0){
    int count_right =0;
    int count_right_behind = 0;
    for(auto car: otherCars){
      if(mycar.lane_at_right == car.lane){
        count_right+=1;
        gap_right = car.s - mycar.s;
        //The number of cars behind right
        if(gap_right<0){
          count_right_behind +=1;
        }
        if( gap_right > 0 && gap_right < min_gap){
          min_gap = gap_right;
          this -> adjacent_front_v = car.v;
          this -> adjacent_front_right_gap = min_gap;

        }
      }
      //Occurs when the car is in the right lane.
      if(mycar.lane_at_right == LaneLoc::NONE){
        this -> adjacent_front_right_gap = 0.0;
      }
    }
    //If all the right car is behind right, we set the front gap is MAX_OBSERVABLE_DIST.
    if(count_right == count_right_behind){
      this -> adjacent_front_right_gap = MAX_OBSERVABLE_DIST;
    }
  }

  return min_gap;
}

double BehaviorPlanner::get_gap_adjacent_behind(Vehicle& mycar, vector<Vehicle>& otherCars, string direction){
  double gap_left;
  double gap_right;
  double min_gap = 100000000;

  //For left lane
  if(direction.compare("LEFT") == 0){
    int count_left =0;
    int count_left_front = 0;
    for(auto car : otherCars){
      //The number of cars on the left lane
      if(mycar.lane_at_left == car.lane){
        count_left+=1;
        gap_left =  mycar.s - car.s;
        //The number of cars left front lane
        if(gap_left<0){
          count_left_front +=1;
        }
        //Calculate minimum gap.
        if( gap_left > 0 && gap_left < min_gap){
          min_gap = gap_left;
          this -> adjacent_behind_left_gap = min_gap;
        }
      }
      //Occurs when the car is in the left lane.
      if(mycar.lane_at_left == LaneLoc::NONE){
        this -> adjacent_behind_left_gap = 0.0;
      }
    }
    //If all the left cars are left front, we set the behind gap is MAX_OBSERVABLE_DIST.
    if(count_left == count_left_front){
      this -> adjacent_behind_left_gap = MAX_OBSERVABLE_DIST;
    }

  }else if(direction.compare("RIGHT") == 0){
    int count_right =0;
    int count_right_front = 0;
    for(auto car :otherCars){

      if(mycar.lane_at_right == car.lane){
        count_right+=1;
        gap_right = mycar.s - car.s;
        //The number of cars right front
        if(gap_right<0){
          count_right_front +=1;
        }
        if( gap_right > 0 && gap_right < min_gap){
          min_gap = gap_right;
          this -> adjacent_behind_right_gap = min_gap;
        }
      }
      //Occurs when the car is in the right lane.
      if(mycar.lane_at_right == LaneLoc::NONE){
        this -> adjacent_behind_right_gap = 0.0;
      }
    }
    //If all the cars are right cars are right front, we set the behind gap is MAX_OBSERVABLE_DIST.
    if(count_right == count_right_front){
      this -> adjacent_behind_right_gap = MAX_OBSERVABLE_DIST;
    }
  }
  return min_gap;
}



double BehaviorPlanner::lane_change_cost(Vehicle& mycar){
  double cost = 0.0;
  int total_lane = 3;
  //LaneLoc::MIDDLE(1) is the target lane
  cost = W_LC * (1.0 - fabs(LaneLoc::MIDDLE-mycar.lane)/(total_lane-1));
}

double BehaviorPlanner::distance_cost(Vehicle& mycar, string direction){
  double cost = 0.0;
  //TURN LEFT or TURN RIGHT
  if(direction.compare("LEFT") == 0){
    cost = W_DC * ( (MAX_OBSERVABLE_DIST- this->adjacent_front_left_gap)/MAX_OBSERVABLE_DIST);
  }else if(direction.compare("RIGHT") == 0){
    cost = W_DC * ( (MAX_OBSERVABLE_DIST- this->adjacent_front_right_gap)/MAX_OBSERVABLE_DIST);
  }else{
    //KEEP LANE
    cost = W_DC * ( (MAX_OBSERVABLE_DIST- this->front_gap)/MAX_OBSERVABLE_DIST);
  }
  return cost;
}

double BehaviorPlanner::velocity_cost(Vehicle& mycar){
  double cost = 0.0;
  cost = W_VEL * (1.0- (SPEED_LIMIT-mycar.front_v)/SPEED_LIMIT);
  return cost;
}

double BehaviorPlanner::gap_cost(Vehicle& mycar, string direction){
  double cost = 0.0;
  if(direction.compare("LEFT")==0){
    cost =W_GAP * 1/(this->adjacent_front_left_gap);
  }else if(direction.compare("RIGHT")==0){
    cost =W_GAP * 1/(this->adjacent_front_right_gap);
  }else{
    cost = W_GAP * 1/(this->front_gap);
  }
  return cost;
}

double BehaviorPlanner::calc_cost(Vehicle& mycar, vector<Vehicle>& otherCars, string direction){
  double cost = 0.0;

  double gap_front = get_gap(mycar, otherCars);

  //lane_change_cost and velocity_cost are mutual costs for keep lane, change left and change right.
  cost += lane_change_cost(mycar);
  cost += velocity_cost(mycar);


  //Turn Left cost
  if(direction.compare("LEFT")==0){
    double gap_left_front = get_gap_adjacent_front(mycar, otherCars, direction);
    cost += gap_cost(mycar,direction);

    //Additional cost for lane deviation
    if(mycar.lane_at_left == LaneLoc::NONE){
      cost +=100;
    }
  //Turn right cost
  } else if(direction.compare("RIGHT")==0){
    double gap_right_front = get_gap_adjacent_front(mycar, otherCars, direction);
    cost += gap_cost(mycar,direction);
    //Additional cost for lane deviation
    if(mycar.lane_at_right == LaneLoc::NONE){
      cost +=100;
    }
  }else{
    //Keep lane cost
    cost += gap_cost(mycar,direction);
  }
  return cost;
}

Action BehaviorPlanner::get_action(Vehicle& mycar, vector<Vehicle>& otherCars){
  Action action = Action::KEEP_LANE;


  double keep_lane_cost, turn_left_cost, turn_right_cost;
  keep_lane_cost = calc_cost(mycar ,otherCars, "");
  turn_left_cost = calc_cost(mycar ,otherCars, "LEFT");
  turn_right_cost = calc_cost(mycar ,otherCars, "RIGHT");

  //Front gap is caluclated by the cost function
  double front_left_gap = this-> adjacent_front_left_gap;
  double front_right_gap = this-> adjacent_front_right_gap;

  //Only get gap
  double behind_left_gap = get_gap_adjacent_behind(mycar ,otherCars,"LEFT");
  double behind_right_gap = get_gap_adjacent_behind(mycar ,otherCars,"RIGHT");
  cout<<"BEHIND LEFT GAP: "<<this->adjacent_behind_left_gap<<endl;
  cout<<"BEHIND RIGHT GAP: "<<this->adjacent_behind_right_gap<<endl;


  mycar.front_v = this ->front_v;
  mycar.front_gap = this -> front_gap;



  cout<<"FRONT GAP: "<<this->front_gap<<" FRONT LEFT GAP: "<<front_left_gap<<"  FRONT RIGHT GAP: "<<front_right_gap<<endl;

  //If front gap is the smallest, the keep lane will get aditional cost
  if(this->front_gap < front_left_gap && this->front_gap < front_right_gap){
    keep_lane_cost += 0.19;
  }

  //Change lane cost(constant cost)
  turn_left_cost +=0.20;
  turn_right_cost+=0.20;

  //If there are enough gap in front, it will reduce the cost(constant cost)
  if(this->front_gap > 50){
    keep_lane_cost -= 0.05;
  }
  if(front_left_gap > 50){
    turn_left_cost -= 0.05;
  }
  if(front_right_gap > 50){
    turn_right_cost -= 0.05;
  }

  //If the car try to change lane with low speed, it will get additional constant cost
  if(mycar.current_state_s.v < MIN_LANE_CHANGE_SPEED){
    turn_left_cost +=0.15;
    turn_right_cost += 0.15;
  }

  cout<< "KEEP_LANE_COST: "<<keep_lane_cost<<endl;
  cout<<"TURN_LEFT COST: "<<turn_left_cost<<endl;
  cout<<"TURN RIGHT COST: "<<turn_right_cost<<endl;


  //Feasibility Cheack//

  //If there are enough gap for all lanes, the car will choose keep lane for SAFETY
  if(this->front_gap > SAFE_GAP && front_left_gap > SAFE_GAP && front_right_gap > SAFE_GAP){
    action = Action::KEEP_LANE;
  }
  if(keep_lane_cost<= turn_left_cost && keep_lane_cost <= turn_right_cost){
    cout<<"KEEP LANE"<<endl;
    action = Action::KEEP_LANE;

  //if the cost is the smallest and the behind left buffer is large enough, the car will change to the left lane.
  }else if(turn_left_cost < keep_lane_cost && turn_left_cost <= turn_right_cost && behind_left_gap > SAFE_BEHIND_BUFFER){
    cout<<"TURN LEFT"<<endl;
    action = Action::TURN_LEFT;

  //if the cost is the smallest and the behind right buffer is large enough, the car will change to the right lane.
  }else if(turn_right_cost < keep_lane_cost && turn_right_cost < turn_left_cost && behind_right_gap > SAFE_BEHIND_BUFFER){
    cout<<"TURN RIGHT"<<endl;
    action = Action::TURN_RIGHT;
  }
  return action;
}

