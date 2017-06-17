#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "matplotlibcpp.h"

// for convenience
using json = nlohmann::json;
namespace plt = matplotlibcpp;

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

// Compute the first derivation of the polynomial:
double poly_deriv(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i-1)*i;
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
  std::vector<double> x_vals;
  std::vector<double> y_vals;
  std::vector<double> psi_vals;
  std::vector<double> v_vals;
  std::vector<double> cte_vals;
  std::vector<double> epsi_vals;
  std::vector<double> delta_vals;
  std::vector<double> a_vals;
  std::vector<double> costs;

  bool initialized = false;
  bool monitoring = false; // determine if we want to monitor the data
  int iters = 0;          // counter of the number of iterations to control when to show the data

  h.onMessage([&mpc,
               &x_vals,
               &y_vals,
               &psi_vals,
               &v_vals,
               &cte_vals,
               &epsi_vals,
               &delta_vals,
               &a_vals,
               &initialized,
               &monitoring,
               &iters,
               &costs
               ](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

          iters += 1; // for tracking monitoring display

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * DONE: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // convert to vehicle's local coordinates, then all computation is in terms of the local coordinates
          Eigen::VectorXd ptsx_local(ptsx.size());
          Eigen::VectorXd ptsy_local(ptsy.size());

          for (size_t i = 0; i < ptsx.size(); i++) {
            ptsx_local[i] = (ptsx[i] - px)*cos(-psi) - (ptsy[i] - py)*sin(-psi);
            ptsy_local[i] = (ptsx[i] - px)*sin(-psi) + (ptsy[i] - py)*cos(-psi);
          }

          // with local coordinates,
          double px_local = 0;
          double py_local = 0;
          double psi_local = 0;

          auto coeffs = polyfit(ptsx_local, ptsy_local, 3);
          double cte = polyeval(coeffs, px_local) - py_local;
          double epsi = psi_local -atan(poly_deriv(coeffs, px_local));

          Eigen::VectorXd state(6);
          state << px_local, py_local, psi_local, v, cte, epsi;

          if (monitoring && !initialized) {
            initialized = true;
            x_vals = {state[0]};
            y_vals = {state[1]};
            psi_vals = {state[2]};
            v_vals = {state[3]};
            cte_vals = {state[4]};
            epsi_vals = {state[5]};
            delta_vals = {};
            a_vals = {};
          }

          auto output = mpc.Solve(state, coeffs);

          if (monitoring && initialized) {
            x_vals.push_back(output.next_vars[0]);
            y_vals.push_back(output.next_vars[1]);
            psi_vals.push_back(output.next_vars[2]);
            v_vals.push_back(output.next_vars[3]);
            cte_vals.push_back(output.next_vars[4]);
            epsi_vals.push_back(output.next_vars[5]);

            delta_vals.push_back(output.next_vars[6]);
            a_vals.push_back(output.next_vars[7]);
          }

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          double steer_value = -output.next_vars[6]/0.436332; // 0.436332 = rad2deg(25), must negate the delta computed to make the drive stable, don't why yet

          double throttle_value = output.next_vars[7];

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = output.predicted_ptsx; // mpc_x_vals;
          msgJson["mpc_y"] = output.predicted_ptsy; // mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          for (int i = 1; i < ptsx_local.size(); i++) { // Usually the first waypoint is behind the car not helpful, ignore it
            next_x_vals.push_back(ptsx_local[i]);
            next_y_vals.push_back(ptsy_local[i]);
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
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
    if (monitoring && (iters == 500)) {// show the data curves
      // Plot values
      // NOTE: feel free to play around with this.
      // It's useful for debugging!
      plt::subplot(3, 1, 1);
      plt::title("CTE");
      plt::plot(cte_vals);
      plt::subplot(3, 1, 2);
      plt::title("Delta (Radians)");
      plt::plot(delta_vals);
      plt::subplot(3, 1, 3);
      plt::title("Velocity");
      plt::plot(v_vals);
      plt::show();

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
