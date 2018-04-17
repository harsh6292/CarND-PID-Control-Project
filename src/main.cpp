#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int arg_count, char* argv[])
{
  uWS::Hub h;

  bool isConnectedToSimulator = false;

  // Constant variable to define if twiddle is enabled
  const bool isTwiddleEnabled = false;

  // Max steps to run twiddle and then reset
  const int MAX_STEP_TWIDDLE = 700;

  int timestep = 1;

  std::ofstream cte_file;

  PID steerPid;
  PID speedPid;
  // TODO: Initialize the pid variable.

  // Default values of constants
  double kp = 0.25;
  double ki = 0.0001567;
  double kd = 1.783;

  //std::cout<<"Argument count: "<<arg_count<<std::endl;

  // Read constant values from command line args if they are present
  if (arg_count > 3) {
    if (argv[1] != NULL) {
      kp = atof(argv[1]);
    }

    if (argv[2] != NULL) {
      ki = atof(argv[2]);
    }

    if (argv[3] != NULL) {
      kd = atof(argv[3]);
    }
  }

  std::cout<<"Initial values of Kp: "<<kp<<", Ki: "<<ki<<", Kd: "<<kd<<std::endl;

  steerPid.Init(kp, ki, kd);

  speedPid.Init(0.5, 0.0, 0.65);

  //cte_file.open("cte_file.txt", std::ios::out);
  //cte_file<<"Time Step    CTE\n";

  h.onMessage([&steerPid, &speedPid, &cte_file, &timestep](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          // Update the PID controller taking into account new CTE value
          steerPid.UpdateError(cte);

          // Get the new steering value from PID
          steer_value = steerPid.TotalError();

          if (steer_value < -1.0) {
            steer_value = -1.0;
          } else if (steer_value > 1.0) {
            steer_value = 1.0;
          }


          speedPid.UpdateError(speed - 30.0);
          speed = speedPid.TotalError();

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          //std::cout << "Speed: " << speed << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = speed;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);


          //cte_file << timestep << " " << cte << "\n";

          // Start checking for twiddle here
          if (isTwiddleEnabled && steerPid.getNumOfSteps() % MAX_STEP_TWIDDLE == 0) {
            steerPid.twiddle();

            std::cout << "################################################" << std::endl;
            std::cout << "Max step for twiddle reached with params" << std::endl;
            double new_kp = steerPid.tau[0];
            double new_ki = steerPid.tau[1];
            double new_kd = steerPid.tau[2];
            std::cout << "Params: Kp: " << new_kp << ", Ki: " << new_ki << ", Kd: " << new_kd << std::endl;
            std::cout << "Resetting simulator with above initial params now" << std::endl;

            steerPid.Init(new_kp, new_ki, new_kd);

            // Reset the simulator now
            std::string reset_msg = "42[\"reset\", {}]";
            std::cout << reset_msg << std::endl;
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
          }

          timestep += 1;
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onConnection([&h, &isConnectedToSimulator, &cte_file](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;

    if (isConnectedToSimulator) {
        //cte_file.close();
    } else {
        isConnectedToSimulator = true;
    }
  });

  h.onDisconnection([&h, &cte_file](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    //cte_file.close();
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
