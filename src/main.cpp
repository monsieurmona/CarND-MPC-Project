#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
      std::cout << sdata << std::endl;
      if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
         string s = hasData(sdata);
         if (s != "") {
            auto j = json::parse(s);
            string event = j[0].get<string>();
            if (event == "telemetry") {
               // j[1] is the data JSON object

               // desired track
               std::vector<double> ptsx = j[1]["ptsx"];
               std::vector<double> ptsy = j[1]["ptsy"];

               // car state
               double px = j[1]["x"];
               double py = j[1]["y"];
               double psi = j[1]["psi"];
               double v = j[1]["speed"];

               MPC::Coordinates track(ptsx, ptsy);
               track.transformToCarPerspective(px, py, psi);
               MPC::PolynomialCoefficient polyCoeffs = track.polyfit();

               // The car position and angle is now 0.0, as the track
               // is now transformed to car coordinate system
               //
               // MPC::StateVars carState(px, py, psi, v, polyCoeffs);
               MPC::StateVars carState(0, 0, 0, v, polyCoeffs);

               /**
                * Calculate steering angle and throttle using MPC.
                * Both are in between [-1, 1].
                */

               const MPC::Actuation actuation = mpc.solve(carState, polyCoeffs);

               const double steer_value = actuation[0];
               const double throttle_value = actuation[1];

               json msgJson;
               // NOTE: Remember to divide by deg2rad(25) before you send the
               //   steering value back. Otherwise the values will be in between
               //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
               msgJson["steering_angle"] = steer_value/(deg2rad(25));
               msgJson["throttle"] = throttle_value;

               // Display the MPC predicted trajectory

               //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
               // the points in the simulator are connected by a Green line
               const size_t nMpcCoordinates = MPC::m_nSteps - 1;
               std::array<double, nMpcCoordinates> mpcX;
               std::array<double, nMpcCoordinates> mpcY;
               std::copy_n(actuation.begin() + 2, nMpcCoordinates, mpcX.begin());
               std::copy_n(actuation.begin() + 2 + nMpcCoordinates, nMpcCoordinates, mpcY.begin());

               msgJson["mpc_x"] = mpcX;
               msgJson["mpc_y"] = mpcY;

               // Display the waypoints/reference line
               /**
                *   add (x,y) points to list here, points are in reference to
                *   the vehicle's coordinate system the points in the simulator are
                *   connected by a Yellow line
                */
               const size_t nRefLinePoints = 100;
               std::array<double, nRefLinePoints> refLinePointsX;
               std::array<double, nRefLinePoints> refLinePointsY;

               double x = 0;
               for ( size_t i = 0; i < nRefLinePoints; ++i, x += 3)
               {
                  refLinePointsX[i] = x;
                  refLinePointsY[i] = MPC::polyeval(polyCoeffs, x);
               }

               msgJson["next_x"] = refLinePointsX;
               msgJson["next_y"] = refLinePointsY;

               auto msg = "42[\"steer\"," + msgJson.dump() + "]";
               std::cout << msg << std::endl;
               // Latency
               // The purpose is to mimic real driving conditions where
               //   the car does actuate the commands instantly.
               //
               // Feel free to play around with this value but should be to drive
               //   around the track with 100ms latency.
               //
               // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
               std::this_thread::sleep_for(std::chrono::milliseconds(100));
               ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }  // end "telemetry" if
         } else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
         }
      }  // end websocket if
   }); // end h.onMessage

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
