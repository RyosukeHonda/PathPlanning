#ifndef JMT_H_
#define JMT_H_

#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helper.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

class JMT{
public:
    JMT(State start, State end, double T);
    double jmt_get_val(double T);
    std::vector<double> coeffs;
};


#endif // JMT_H_
