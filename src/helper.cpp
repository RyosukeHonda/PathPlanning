#include "helper.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d, double a, double d_dot, double d_ddot){

  this->id = id;
  this->x  = x;
  this->y  = y;
  this->vx = vx;
  this->vy = vy;
  this->s  = s;
  this->d  = d;
  this->v  = sqrt(vx*vx + vy*vy);
  this->a  = a;
  this->d_dot = d_dot;
  this->d_ddot = d_ddot;
}

Vehicle::~Vehicle();

vector<double> Vedhicle::state_at(double t){
    //s movement
    double s = this->s + this->v * t + this->a * t * t / 2;
    double v = this->v + this->a * t;

    // d movement
    double d = this->d + this->d_dot * t + this->d_ddot * t * t / 2;
    double d_dot = this->d_dot + this->d_ddot * t;

    return {s, v, this->a, d, d_dot, this->d_ddot};
}

double logistic(double x){
    return 2.0/(1+exp(-x))-1.0;
}


double f_val(double t,vector<double> coefficients){
    double total = 0.0;
    for(int i = 0; i< coefficients.size();i++){
        total += coefficients[i]*pow(t,i);
        //cout<<"coef: "<<coefficients[i]<<" t: "<<t<<" t**i: "<<pow(t,i)<<endl;
    }
    return total;
}

vector<double> differentiate(vector<double> coefficients){
    vector<double> new_cos ={};
    vector<double>::iterator it = coefficients.begin();
    int count =0;
    it++;
    while(it != coefficients.end()){
        new_cos.push_back((*it)*(count+1));
        count+=1;
        it++;
    }
    return new_cos;
}
