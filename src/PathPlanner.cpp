#include "PathPlanner.h"
#include <vector>
#include <map>
#include <string>
#include <random>


struct Goal{
    vector<double> s;
    vector<double> d;
    double t;
};

struct Trajectory{
    vector<double> s_coeffs;
    vector<double> d_coeffs;
    double t;
};


PathPlanner::PathPlanner(double x, double y, double s, double d, double yaw, double speed){

    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->speed = speedl;
    state = "CS";
    max_acceleration = -1;
}

PathPlanner::~PathPlanner(){}

constexpr double pi() { return M_PI; }
double deg2rad2(double x) { return x * pi() / 180; }
double rad2deg2(double x) { return x * 180 / pi(); }


vector<double> PathPlanner::JMT(vector< double> start, vector <double> end, double T)
{
    /*
     Calculate the Jerk Minimizing Trajectory that connects the initial state
     to the final state in time T.

     INPUTS

     start - the vehicles start location given as a length three array
     corresponding to initial values of [s, s_dot, s_double_dot]

     end   - the desired end state for vehicle. Like "start" this is a
     length three array.

     T     - The duration, in seconds, over which this maneuver should occur.

     OUTPUT
     an array of length 6, each value corresponding to a coefficent in the polynomial
     s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

     EXAMPLE

     > JMT( [0, 10, 0], [10, 10, 0], 1)
     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
     */

    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
    3*T*T, 4*T*T*T,5*T*T*T*T,
    6*T, 12*T*T, 20*T*T*T;

    MatrixXd B = MatrixXd(3,1);
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
    end[1]-(start[1]+start[2]*T),
    end[2]-start[2];

    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai*B;

    vector <double> result = {start[0], start[1], .5*start[2]};
    for(int i = 0; i < C.size(); i++)
    {
        result.push_back(C.data()[i]);
    }

    return result;

}

vector<string> PathPlanner::fsm_s(string state){
    if(state.compare("KL")==0){
        vector<string> str = {"PLCL","PLCR","KL"};
        return str;
    }else if(state.compare("LCL")==0){
        vector<string> str = {"LCL","KL"};
        return str;
    }else if(state.compare("LCR")==0){
        vector<string> str = {"LCR","KL"};
        return str;
    }else if(state.compare("PLCL")==0){
        vector<string> str = {"PLCL","LCL","KL"};
        return str;
    }else if(state.compare("PLCR")==0){
        vector<string> str = {"PLCR","LCR","KL"};
        return str;
    }
    vector<string> str = {"PLCR","PLCL","KL"};
    return str;
}

double PathPlanner::calculate_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
  double total_cost =0;
  double time_diff_cost = time_diff_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);
  double s_diff_cost = s_diff_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);
  double d_diff_cost = d_diff_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);
  double collision_cost = collision_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);
  double buffer_cost = buffer_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);
  double stay_in_road_cost = stay_in_road_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);
  double exceeds_speed_limit_cost = exceeds_speed_limit_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);
  double efficiency_cost = efficiency_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);
  double total_accel_cost = total_accel_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);
  double max_acc_cost = max_acc_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);
  double max_jerk_cost = max_jerk_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);
  double total_jerk_cost = total_jerk_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions);

  vector<double> w = {1.0,1.0,1.0,1.0,1.0,
                      1.0,1.0,1.0,1.0,1.0,
                      1.0,1.0};
  total_cost = w[0] * time_diff_cost + w[1] * s_diff_cost + w[2] * d_diff_cost + w[3] * collision_cost + w[4] * buffer_cost +
               w[5] * stay_in_road_cost + w[6] * exceeds_speed_limit_cost + w[7] * efficiency_cost + w[8] * total_accel_cost+
               w[9] * max_acc_cost + w[10] * max_jerk_cost + w[11] * total_jerk_cost;
}

vector<double> PathPlanner::perturb_goal(vector<double> goal,vector<double> sigma){
    default_random_engine gen;
    normal_distribution<double> x(goal[0],sigma[0]);
    normal_distribution<double> x_d(goal[1],sigma[1]);
    normal_distribution<double> x_dd(goal[2],sigma[2]);
    double n_x,n_x_d,n_x_dd;
    n_x = x(gen);
    n_x_d = x_d(gen);
    n_x_dd = x_dd(gen);
    return {n_x,n_x_d,n_x_dd};
}

Trajectory PathPlanner::PTG(vector<double> start_s, vector<double> start_d, int target_vehicle, vector<double> delta, double T, vector<Vehicle> predictions){
  /*
    Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS (global).

    arguments:
     start_s - [s, s_dot, s_ddot]

     start_d - [d, d_dot, d_ddot]

     target_vehicle - id of leading vehicle (int) which can be used to retrieve
       that vehicle from the "predictions" dictionary. This is the vehicle that
       we are setting our trajectory relative to.

     delta - a length 6 array indicating the offset we are aiming for between us
       and the target_vehicle. So if at time 5 the target vehicle will be at
       [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal
       state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a
       goal of "follow 10 meters behind and 4 meters to the right of target vehicle"

     T - the desired time at which we will be at the goal (relative to now as t=0)

     predictions - dictionary of {v_id : vehicle }. Each vehicle has a method
       vehicle.state_in(time) which returns a length 6 array giving that vehicle's
       expected [s, s_dot, s_ddot, d, d_dot, d_ddot] state at that time.

    return:
     (best_s, best_d, best_t) where best_s are the 6 coefficients representing s(t)
     best_d gives coefficients for d(t) and best_t gives duration associated w/
     this trajectory.
    */
     vector<Goal> all_goals;
     double timestep = 0.5;
     double t = T - 4 + timestep;
     vector<double> SIGMA_S = {10.0, 4.0, 2.0};
     vector<double> SIGMA_D = {1.0,1.0,1.0};
     while(t <= T + 4 * timestep){
        Goal goal;
        vector<double> target_state = predictions[target_vehicle].state_in(t) + delta;
        vector<double> goal_s = {target_state[0],target_state[1],target_state[2]};
        vector<double> goal_d = {target_state[3],target_state[4],target_state[5]};
        goal.s = goal_s;
        goal.d = goal_d;
        goal.t = t;
        all_goals.push_back(goal);
        for(int i = 0; i < N_SAMPLES ; i++){
          Goal purturbed;
          // TODO IMPLMENT PURTURBED GOAL
          purturbed.s = perturb_goal(goal_s, SIGMA_S);
          purturbed.d = perturb_goal(goal_d, SIGMA_D);
          purturbed.t = t;
          all_goals.push_back(purturbed);
        }
        t += timestep;
     }

     Trajectory trajectory;
     Trajectory best_traj;
     double best_cost = 99999999;
     double cost;
     for(int i = 0; i < all_goals.size(); i++){
         vector<double> s_goal = all_goals[i].s;
         vector<double> d_goal = all_goals[i].d;
         double t = all_goals[i].t;
         vector<double> s_coefficients = JMT(start_s, s_goal, t);
         vector<double> d_coefficients = JMT(start_d, d_goal, t);

         trajectory.s_coeffs = s_coefficients;
         trajectory.d_coeffs = d_coefficients;
         trajectory.t = t;
         cost = calculate_cost(trajectory, target_vehicle, delta, T, predictions);
         if(cost < best_cost){
             best_cost = cost;
             best_traj = trajectory;
         }
     }
     return best_traj;
}



