#ifndef HELPER_H
#define HELPER_H


class Vehicle
{
    public:
    
    Vehicle(int id, double x, double y, double vx, double vy, double s, double d, double a, double d_dot, double d_ddot);
    
    ~Vehicle();
    
    // car's unique ID
    int id;
    
    // car's x position in map coordinates
    double x;
    
    // car's y position in map coordinates,
    double y;
    
    // car's x velocity in m/s
    double vx;
    
    // car's y velocity in m/s
    double vy;
    
    // car's s position in frenet coordinates
    double s;
    
    // car's d position in frenet coordinates
    double d;
    
    // car's total velocity in m/s
    double v; // s_dot
    
    // car's total acceleration in m/s/s
    double a; // s_double_dot
    
    // car's d velocity in m/s
    double d_dot;
    
    // car's d acceleration in m/s/s
    double d_ddot;
    
    vector<double> state_at(double t);
};

double logistic(double x);


double f_val(double t,vector<double> coefficients);

vector<double> differentiate(vector<double> coefficients);

#endif
