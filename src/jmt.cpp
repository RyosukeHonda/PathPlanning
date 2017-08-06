#include "jmt.h"
#include <cmath>
#include <vector>

using namespace std;


// TODO - complete this function
JMT::JMT(State start, State end, double T)
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
    MatrixXd A = MatrixXd(3,3);
    A << T*T*T, T*T*T*T,T*T*T*T*T,
         3*T*T, 4*T*T*T,5*T*T*T*T,
         6*T, 12*T*T, 20*T*T*T;
    MatrixXd B = MatrixXd(3,1);
    B << end.p-(start.p+start.v*T+0.5*start.a*T*T),
         end.v- (start.v+ start.a*T),
         end.a- start.a;
    MatrixXd C = A.inverse()*B;
    vector <double> result = {start.p, start.v, .5*start.a};
  for(int i = 0; i < C.size(); i++)
  {
      result.push_back(C.data()[i]);
  }
    this-> coeffs = result;
}

double JMT::jmt_get_val(double T){
    vector<double> c = this-> coeffs;
    double val = 0.0;
    val = c[0] * 1.0 + c[1] * T + c[2] * pow(T,2) + c[3] * pow(T,3) + c[4] * pow(T,4) + c[5] * pow(T,5);
    return val;
}

