#include "pathconverter.h"

using namespace std;

PathConverter::PathConverter(std::string file_path, const double distance) {

  vector<double> xs;
  vector<double> ys;
  vector<double> ss;
  vector<double> dxs;
  vector<double> dys;

  double x, y, s, dx, dy;
  double x0, y0, dx0, dy0;

  bool first = true;

  string line;
  ifstream in_file(file_path.c_str(), ifstream::in);

  if (!in_file.is_open()) {
    cerr << "Error in opening the input data: " << file_path << endl;
    exit(EXIT_FAILURE);
  }

  // Load information from file to memory
  while (getline(in_file, line)) {

    istringstream iss(line);

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;

    xs.push_back(x);
    ys.push_back(y);
    ss.push_back(s);
    dxs.push_back(dx);
    dys.push_back(dy);

    //store initial points to roop
    if (first) {
      x0 = x;
      y0 = y;
      dx0 = dx;
      dy0 = dy;
      first = false;
    }
  }

  if (in_file.is_open()) {
    in_file.close();
  }

  //connect initial points to the last points for roop
  xs.push_back(x0);
  ys.push_back(y0);
  ss.push_back(distance);
  dxs.push_back(dx0);
  dys.push_back(dy0);

  // Uses the loaded information to fit cubic polynomial curves
  this->x_spline.set_points(ss, xs);
  this->y_spline.set_points(ss, ys);
  this->dx_spline.set_points(ss, dxs);
  this->dy_spline.set_points(ss, dys);
  this->distance = distance;
}

vector<double> PathConverter::convert_sd_to_xy(double s, double d){

  double mod_s = fmod(s, this->distance);
  double x_edge = this->x_spline(mod_s);
  double y_edge = this->y_spline(mod_s);
  double dx = this->dx_spline(mod_s);
  double dy = this->dy_spline(mod_s);

  double x = x_edge + dx * d;
  double y = y_edge + dy * d;

  return {x, y};
}

XYPoints PathConverter::generate_path(JMT jmt_s, JMT jmt_d, double t, int n){

  vector<double> xs;
  vector<double> ys;
  vector<double> p;

  for (int i = 0; i < n; i++) {

    double s = jmt_s.jmt_get_val(i * t);
    double d = jmt_d.jmt_get_val(i * t);

    vector<double> p = this -> convert_sd_to_xy(s, d);

    xs.push_back(p[0]);
    ys.push_back(p[1]);
  }

  XYPoints path = {xs, ys, n};

  return path;
}

