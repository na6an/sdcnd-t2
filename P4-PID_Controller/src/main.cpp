#include "PID.h"
#include "json.hpp"
#include <iostream>
#include <math.h>
#include <uWS/uWS.h>
#include <vector>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos)
  {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() //int argc, char *argv[])
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  double Kp = -0.06; // atof(argv[1]);
  double Ki = -1E-05; // atof(argv[2]);
  double Kd = -5; // atof(argv[3]);
  pid.Init(Kp, Ki, Kd);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    double cte_p = 0;
    // "42" at the start of the message means there's a websocket
    // message event. The 4 signifies a websocket message The 2
    // signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2'){
      auto s = hasData(std::string(data).substr(0, length));
      if (s != ""){
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry"){
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          //if (cte_p )
          //cte_p = cte;
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          pid.UpdateError(cte);

          double steer = 2*pid.TotalError()*abs(cte - cte_p);
          //std::vector<double> p;
          //p = pid.twiddle(cte);
          //std::cout << p[0] << "," << p[1] << "," << p[2] << std::endl;
          double th;
          if (abs(cte) > 1){
            th = 0.5 - cte * cte * speed * abs(angle)/ 1000; //0.4 + pid.UpdateError(cte);
          }else{
            th = 0.5 - abs(cte) * speed / 1000;
          }
          cte_p = cte;

          // DEBUG
          std::cout << "CTE: " << cte << " Steer: " << steer
          << " Speed: " << speed <<  " th: " << th << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer;
          msgJson["throttle"] = th;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << "cte:" << cte;
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
