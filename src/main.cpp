#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  pid.Init(0.04, 0.00015, 0.3);
  vector<double> p{0.1, 0.0001, 1.0};
  vector<double> dp{0.01, 0.0001, 0.1};
  int i = 0;
  bool flag_1 = true;    // if flag_1 = true, p[i] += dp[i]
  bool flag_2 = false;   // if flag_2 = true, p[i] -= dp[i]
  bool flag_3 = false;   // if flag_3 = true, means p[i] += dp[i], but error > best_error, so need to set flag_2 = true
  bool twiddle = false;  // twiddle is only false at the beginning or the sumdp < 0.0001
  bool begin = false;     // begin is only true at the beginning
  double total_error = 0.0;
  double best_error;
  int count = 0;   // count how many moves
  int n = 500;     // every iteration will run 500 times

  h.onMessage([&pid, &i, &twiddle, &flag_1, &flag_2, &flag_3, &best_error, &total_error, &count, &begin, &p, &dp, &n]
             (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *       Maybe use another PID controller to control the speed!
           */

          if (begin && count < n){
            // calculate the initial error, and set it as best error
            total_error += cte * cte;
            count += 1;
            if (count == n){
              best_error = total_error / n;
              total_error = 0;
              begin = false;
            }
          }
          else if (!begin && !twiddle){
            count = 0;
            //twiddle = true;
          }

          if (twiddle){
            if (count == n){
              count = 0;
            }
            if (flag_1){
              p[i] += dp[i];
              std::cout<< "first p[i] = " << p[i] << " i = " << i << std::endl;
              if (i == 0){
                pid.Kp = p[i];
              }
              else if (i ==1){
                pid.Ki = p[i];
              }
              else if (i == 2){
                pid.Kd = p[i];
              }
              flag_1 = false;
              flag_3 = true;
            }
            else if (flag_2){
              p[i] -= 2 * dp[i];
              std::cout << "second p[i] = " << p[i] << " i = " << i << std::endl;
              if (i == 0){
                pid.Kp = p[i];
              }
              else if (i ==1){
                pid.Ki = p[i];
              }
              else if (i == 2){
                pid.Kd = p[i];
              }
              flag_2 = false;
            }
            if (count < n){
              total_error += cte * cte;
              count += 1;
              if (count == n){
                double error = total_error / n;
                total_error = 0;
                if (error < best_error){
                  best_error = error;
                  dp[i] *= 1.1;
                  flag_1 = true;
                  std::cout << "best p[i] = " << p[i] << " i = " << i << std::endl;
                  if (i < 2){
                    i += 1;
                  }
                  else{
                    i = 0;
                  }
                }
                else if (error > best_error && flag_3){
                  flag_2 = true;
                  flag_3 = false;
                }
                else{
                  p[i] += dp[i];
                  if (i == 0){
                    pid.Kp = p[i];
                  }
                  else if (i ==1){
                    pid.Ki = p[i];
                  }
                  else if (i == 2){
                    pid.Kd = p[i];
                  }
                  dp[i] *= 0.9;
                  flag_1 = true;
                  std::cout << "current p[i] = " << p[i] << " i = " << i << " dp[i] = " << dp[i] <<std::endl;
                  if (i < 2){
                    i += 1;
                  }
                  else{
                    i = 0;
                  }
                }
              }
            }
          }

          double sumdp = dp[0] + dp[1] + dp[2];
          if (sumdp < 0.001){
            twiddle = false;
            std::cout << "best parameter: " << p[0] << "," << p[1] << "," << p[2] << std::endl;
          }

          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          if (count == 1){
            std::cout << "parameter: " << pid.Kp << "," << pid.Ki << "," << pid.Kd << std::endl;
          }

          // DEBUG
          //std::cout << "CTE: " << cte <<" Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          if (count == n){
            string msg = "42[\"reset\",{}]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else{
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
