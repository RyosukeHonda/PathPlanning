#include "helper.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


double time_diff_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    //Penalizes trajectories that span a duration which is longer or
    //shorter than the duration requested.
    double t = traj.t;
    return logistic(float(abs(t-T))/T);
}

double s_diff_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    //Penalizes trajectories whose s coordinate (and derivatives)
    //differ from the goal.
    vector<double> s = traj.s_coeffs;
    double t = traj.t;
    vector<double> target;
    target = predictions[target_vehicle].state_in(t);
    target = target + delta;
    vector<double> S;
    vector<double> s_dot = differentiate(s);
    vector<double> s_ddot = differentiate(s_dot);
    S = {f_val(T,s),f_val(T,s_dot),f_val(T,s_ddot)};

    double cost = 0;
    double diff;
    for(int i = 0; i<S.size();i++){
        diff = abs(S[i]-target[i]);
        cost += logistic(diff/SIGMA_S[i]);
    }
    return cost;
}

double d_diff_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    //Penalizes trajectories whose d coordinate (and derivatives)
    //differ from the goal.
    vector<double> d = traj.d_coeffs;
    double t = traj.t;
    vector<double> target;
    target = predictions[target_vehicle].state_in(t);
    target = target + delta;  //{s,s_dot,s_ddot,d,d_dot,d_ddot}
    vector<double> D;
    vector<double> d_dot = differentiate(d);
    vector<double> d_ddot = differentiate(d_dot);
    D = {f_val(T,d),f_val(T,d_dot),f_val(T,d_ddot)};

    double cost = 0;
    double diff;
    for(int i = 0; i<D.size();i++){
        diff = abs(D[i]-target[i+3]);  //target[i+3]:{d,d_dot,d_ddot}
        cost += logistic(diff/SIGMA_D[i]);
    }
    return cost;
}

double collision_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    //Binary cost function which penalizes collisions.
    double nearest;
    nearest = nearest_approach_to_any_vehicle(traj,predictions);
    if(nearest < 2 * VEHICLE_RADIUS){
        return 1.0;
    }else{
        retutn 0.0;
    }
    return 0.0;
}


double buffer_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    //Penalizes getting close to other vehicles.
    double nearest;
    nearest = nearest_approach_to_any_vehicle(traj, predictions);
    return logistic(2 * VEHICLE_RADIUS / nearest);
}

double stays_in_road_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    return 0.0;
}

double exceeds_speed_limit_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    return 0.0;
}

double efficiency_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    vector<double> s = traj.s_coeffs;
    double t = traj.t;
    double avg_v = float(f_val(t,s));
    vector<double> targ = predictions[target_vehicle];
    double targ_v = float(targ[0])/t;
    return logistic(2 * float(targ_v - avg_v) / avg_v);
}

double  total_accel_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    vector<double> s = traj.s_coeffs;
    vector<double> d = traj.d_coeffs;

    vector<double> s_dot = differentiate(s);
    vector<double> s_ddot = differentiate(s_dot);
    double total_acc = 0;
    double dt = T / 100.0;
    double t,acc;
    for(int i = 0; i < 100; i++){
        t = dt * i;
        acc = f_val(t, s_ddot);
        total_acc += abs(acc * dt);
    }
    double acc_per_second = total_acc / T;
    return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC);
}

double max_acc_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    vector<double> s = traj.s_coeffs;
    vector<double> d = traj.d_coeffs;

    vector<double> s_dot = differentiate(s);
    vector<double> s_ddot = differentiate(s_dot);
    double max_acc =0;
    double all_accs;
    for(int i = 0; i< 100; i++){
        all_accs = f_val(T,s_ddot) / 100 * i;
        if(abs(all_accs)>max_acc){
            mac_acc = abs(all_accs);
        }
    }
    if(max_acc > MAX_ACCEL){
        return 1.0;
    }else{
        return 0;
    }
    return 0;
}

double max_jerk_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    vector<double> s = traj.s_coeffs;
    vector<double> d = traj.d_coeffs;

    vector<double> s_dot = differentiate(s);
    vector<double> s_ddot = differentiate(s_dot);
    vector<double> jerk = differentiate(s_ddot);

    double max_jerk =0;
    double all_jerks;
    for(int i = 0; i< 100; i++){
        all_jerks = f_val(T,jerk) / 100 * i;
        if(abs(all_jerks)>max_jerk){
            mac_jerk = abs(all_jerks);
        }
    }
    if(max_jerk > MAX_JERK){
        return 1.0;
    }else{
        return 0;
    }
    return 0;
}

double  total_jerk_cost(Trajectory traj,int target_vehicle,vector<double> delta, double T, vector<Vehicle> predictions){
    vector<double> s = traj.s_coeffs;
    vector<double> d = traj.d_coeffs;

    vector<double> s_dot = differentiate(s);
    vector<double> s_ddot = differentiate(s_dot);
    vector<double> jerk = differentiate(s_ddot);

    double total_jerk = 0;
    double dt = T / 100.0;
    double t,j;
    for(int i = 0; i < 100; i++){
        t = dt * i;
        j = f_val(t, jerk);
        total_jerk += abs(j * dt);
    }
    double jerk_per_second = total_jerk / T;
    return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC);
}
























