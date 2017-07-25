#include "helper.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
