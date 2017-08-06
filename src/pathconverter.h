#ifndef PATHCONVERTER_H_
#define PATHCONVERTER_H_

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "spline.h"
#include "helper.h"
#include "jmt.h"



class PathConverter{

  public:
    //Import waypoints data into memory
    PathConverter(std::string file_path, double distance);
    // from frenet coordinate to cartesian coordinate
    std::vector<double> convert_sd_to_xy(double s, double d);
    // Generate path from the calculated jerk minimizing trajectory 
    XYPoints generate_path(JMT jmt_s, JMT jmt_d, double t,int n);

  private:
   double distance;
   tk::spline x_spline;
   tk::spline y_spline;
   tk::spline dx_spline;
   tk::spline dy_spline;
};

#endif // PATHCONVERTER_H_
