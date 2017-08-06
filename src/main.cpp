#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "pathconverter.h"
#include "vehicle.h"
#include "jmt.h"
#include "behavior_planner.h"
#include "trajectory.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}



int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  cout << "Program Started." << endl;
  bool init = true;

  cout << "Loading map..." << endl;
  PathConverter pathConverter("../data/highway_map.csv", TRACK_DISTANCE);
  cout << "Map loaded..." << endl;

  h.onMessage([&pathConverter, &init](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data
          	auto sensor_fusion = j[1]["sensor_fusion"];

            Vehicle myCar(-1);
            myCar.update_position(car_s, car_d);
            myCar.update_speed(car_speed);
            myCar.get_adjacent_lane();



          //give back our previous plan
            int n = previous_path_x.size();
            //These points are sent to the simulator
            XYPoints XY_points = {previous_path_x, previous_path_y, n};

            if (init) {
                int n = 225;
                double t = n * TIME_INCREMENT;
                double target_speed = 20.0;
                double target_s = myCar.s + 40.0;

                State startState_s = {myCar.s, myCar.v, 0.0};
                State startState_d = {myCar.d, 0.0, 0.0};

                State endState_s = {target_s, target_speed, 0.0};
                State endState_d = {myCar.convert_lane_to_d(myCar.lane), 0.0, 0.0};

                JMT jmt_s(startState_s, endState_s, t);
                JMT jmt_d(startState_d, endState_d, t);

                //get current state of s and d.
                myCar.get_current_state(endState_s, endState_d);
                //Finish initialization
                init = false;
                //Generate path
                XY_points = pathConverter.generate_path(jmt_s, jmt_d, TIME_INCREMENT, n);

             // If the length of data became smaller than PATH_SIZE, get new path again.
             } else if (n < PATH_SIZE) {

            //Other cars on the same direction
            vector<Vehicle> otherCars;

            for (int i = 0; i < sensor_fusion.size(); i++) {

              int id = sensor_fusion[i][0];
              double s = sensor_fusion[i][5];
              double d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];

              Vehicle car(id);
              car.update_position(s, d);
              car.update_speed(sqrt(vx * vx + vy * vy));
              otherCars.push_back(car);
            }

            //Get Behavior plan by calculating cost and by considering feasibility.
            BehaviorPlanner planner;
            //Get action(Keep lane, change left and change right)
            Action action = planner.get_action(myCar, otherCars);
            //Make Trajectory
            Trajectory trajectory(myCar, action);
            //Update the current state for the next path generation
            myCar.get_current_state(trajectory.target_state_s, trajectory.target_state_d);

            //change frenet coordinate(s and d) system to cartitian coordinate system(x and y)
            XYPoints NextXY_points = pathConverter.generate_path(
              trajectory.get_jmt_s(), trajectory.get_jmt_d(), TIME_INCREMENT, NUMBER_OF_POINTS);

            NextXY_points.n = NUMBER_OF_POINTS;

            // Append path data to the remining data
            XY_points.xs.insert(
              XY_points.xs.end(), NextXY_points.xs.begin(), NextXY_points.xs.end());

            XY_points.ys.insert(
              XY_points.ys.end(), NextXY_points.ys.begin(), NextXY_points.ys.end());

            XY_points.n = XY_points.xs.size();
          }
          	json msgJson;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = XY_points.xs;
            msgJson["next_y"] = XY_points.ys;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}