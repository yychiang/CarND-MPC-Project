#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          vector<double> ptsx_original = j[1]["ptsx"];
          vector<double> ptsy_original = j[1]["ptsy"];

          double px = j[1]["x"];
          double py = j[1]["y"];

          double px_original = j[1]["x"];
          double py_original = j[1]["y"];

          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          cout << px << "," << py << ","<< ptsx[0]<< "," << ptsy[0] <<endl;

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          //Convert the coordinate from map to car
          for (int i = 0; i < ptsx.size(); i++ ) {

            //Translation from world to car
            double new_ptsx = ptsx[i]-px;
            double new_ptsy = ptsy[i]-py;

            //rotation from world to car
            //ptsx[i] = new_ptsx*cos(psi) + new_ptsy*sin(psi);
            //ptsy[i] = new_ptsy*cos(psi) - new_ptsx*sin(psi);
            ptsx[i] = new_ptsx*cos(psi) + new_ptsy*sin(psi);
            ptsy[i] = -new_ptsx*sin(psi) + new_ptsy*cos(psi);
          }

          //In order to use mpc.Solve(), we have to convert all observed waypoints to Eigen::VectorXd
          Eigen::VectorXd ptsx2(6);
          Eigen::VectorXd ptsy2(6);
          ptsx2 << ptsx[0], ptsx[1], ptsx[2], ptsx[3], ptsx[4], ptsx[5];
          ptsy2 << ptsy[0], ptsy[1], ptsy[2], ptsy[3], ptsy[4], ptsy[5];


          //calculating coeffs, cte and epsi and adding latency corrections to all values
          double latency = 0.1;
          const double Lf = 2.67;
          px = v*latency;
          py = 0;
          psi = -v*steer_value/Lf*latency;
          v += throttle_value*latency;
          auto coeffs = polyfit(ptsx2, ptsy2, 3);
          double cte = py-polyeval(coeffs, px);
          double epsi = psi -atan(coeffs[1] + 2*coeffs[2]*px + 3*coeffs[3]*px*px );


          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          //sending state and coeffs to mpc.Solve
          auto vars = mpc.Solve(state, coeffs);



          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          steer_value= -1*vars[6];
          throttle_value=vars[7];
          //throttle_value=0.2;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          //calculating points with the coeffs and set x values using polyeval
          /*
          for (int i = -3; i < 20; i++) {

            int x_value = 2*i;
            double y_value = polyeval(coeffs, x_value);

            next_x_vals.push_back(x_value);
            next_y_vals.push_back(y_value);
          }
           */

          /*
          for (int i=-1;i<8;i++){
            int x_value=3*i;
            double y_value=0;

            next_x_vals.push_back(x_value);
            next_y_vals.push_back(y_value);
          }
           */


          for (int i = 0; i < ptsx.size(); i++ ) {
            double new_ptsx = ptsx[i];

            double y_value = polyeval(coeffs, new_ptsx);
            double new_ptsy = y_value;

            next_x_vals.push_back(new_ptsx);
            next_y_vals.push_back(new_ptsy);

          }




          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
