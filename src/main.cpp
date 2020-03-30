#if 1
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <fstream>
#include <iostream>
#include <string>
#include <uWS/uWS.h>
#include <vector>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // start in lane 1;
  static int lane = 1;  // 0:左车道 1：中间车道 2：左车道
  // have a reference velocity to target
  static double ref_vel = 0.0; // mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
#if 1

          // previous path points size
          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // check the current state of the road
          bool   front_occupied = false;
          double front_speed;
          vector<bool> lane_occupied = {false, false, false};

          // vector<vector<double>> sensor_fusion: is vector of double
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            double check_vx = sensor_fusion[i][3];
            double check_vy = sensor_fusion[i][4];
            double check_s = sensor_fusion[i][5];
            double check_d = sensor_fusion[i][6];
            double check_speed = sqrt(check_vx * check_vx + check_vy * check_vy);

            // if using previous points car project s value out current
            double check_car_s = check_s + (double)prev_size*0.02*check_speed;

            // car in my lane
            if (check_d < (2+4*lane+2) && check_d > (2+4*lane-2)) {
              // check s value greater than mine and s gap
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
                front_occupied = true;
                front_speed = check_speed;
              }
            }

            // Check the three lanes
            for (int k = 0; k < 3; ++k) {
              if (check_d > 4*k && check_d < 4*(k+1)) {
                if ((check_car_s > car_s && (check_car_s - car_s) < 30) ||
                    (check_car_s < car_s && (car_s - check_car_s) < 10)) {
                  lane_occupied[k] = true;
                }
              }
            }
          } // end of sensor data loop

#if IS_PEINT  // IS_PEINT define in helper.h file
          // find my lane
          int my_lane = 0xFF;
          for (int i = 0; i < 3; ++i) {
            if (car_d > 4*i && car_d < 4*(i+1)) {
              my_lane = i;
            }
          }

          static int num = 0;
          static int idx = 0;
          if (num > 5) {
            num = 0;
            idx += 1;
            std::cout << idx << " lane ocp: " <<
                         lane_occupied[0] << "--" <<
                         lane_occupied[1] << "--" <<
                         lane_occupied[2] << "   ";
            std::cout << "my lane: " << my_lane << std::endl;
          } else {
            ++num;
          }
#endif

          // Traffic jam, the ego vehicle change lane and reduce speed
          if (front_occupied) {
            // candidate lanes
            int candidate_lane[2] = {0xFF, 0xFF};
            if (lane == 0) {
              candidate_lane[0] = 1;
            } else if (lane == 1) {
              candidate_lane[0] = 0;
              candidate_lane[1] = 2;
            } else {
              candidate_lane[0] = 1;
            }

            for (int i = 0; i < 2; ++i) {
              if (candidate_lane[i] != 0xFF) {
                if (!lane_occupied[candidate_lane[i]]) {
                  lane = candidate_lane[i];
                  break;
                }
              }
            }

            // reduce ego vehicle sppeed (m/s to mph)
            if (ref_vel > (front_speed * 2.236836) ) {
              ref_vel -= 0.224; // equal to 5m/s2
            }

          } else if (ref_vel < 49.5) {
            ref_vel += 0.224;
          }

          // change to middle lane
          if (lane != 1 && !lane_occupied[1]) {
            lane = 1;
#if IS_PEINT
            std::cout << "Change to Base Lane (Middle Lane)!" << std::endl;
#endif
          }

          vector<double> ptsx;
          vector<double> ptsy;

          // reference x, y, yaw states
          // either we will reference the starting point as where the car is
          // or at the previous paths or pints
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty. use the car starting reference
          if (prev_size < 2) {
            // Use two points that make the paths tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // Use the previous path's and points as starting reference
            // Redefine reference state as previous path and point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // Use two pints that make the path tangent to the previus path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // transformation to this local car's coordinates (some like MPC)
          // ****： the last points of the previous path is at zero
          for (int i = 0; i < ptsx.size(); ++i) {
            // 注意：shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // create a spline
          tk::spline s;
          // set (x, y) points to the spline
          s.set_points(ptsx, ptsy);

          // Start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at
          // our desired reference velocity
          double target_x = 30.0;  // Horizontal distance
          double target_y = s(target_x); // 通过spline曲线得出y的值
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0.0;

          // Fill up the rest of our path planner after filling it with previous points,
          // here we will always output 50 points
          for (int i = 0; i < 50 - previous_path_x.size(); ++i) {
            double N = target_dist / (0.02 * ref_vel / 2.24);  // 2.24: transform mph to m/s
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

#endif

#if 0
          double dist_inc = 0.4;
          for (int i = 0; i < 50; ++i) {
            //// stay in its lane
            double next_s = car_s + (i+1)*dist_inc;
            double next_d = 2;
            auto next_xy = getXY(next_s, next_d, map_waypoints_s,
                                 map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(next_xy[0]);
            next_y_vals.push_back(next_xy[1]);

            //// straight line
            // next_x_vals.push_back(car_x + (dist_inc*i)*cos(deg2rad(car_yaw)));
            // next_y_vals.push_back(car_y + (dist_inc*i)*sin(deg2rad(car_yaw)));
          }
#endif

#if 0
          double pos_x;
          double pos_y;
          double angle;
          int path_size = previous_path_x.size();

          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          if (path_size == 0) {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
          } else {
            pos_x = previous_path_x[path_size];
            pos_y = previous_path_y[path_size];

            double pos_x2 = previous_path_x[path_size];
            double pos_y2 = previous_path_y[path_size];
            angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
          }

          double dist_inc = 0.5;
          for (int i = 0; i < 50 - path_size; ++i) {
            next_x_vals.push_back(pos_x + (dist_inc)*cos(angle+(i+1)*(pi()/100)));
            next_y_vals.push_back(pos_y + (dist_inc)*sin(angle+(i+1)*(pi()/100)));
            pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
            pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
          }
#endif

          /// End TODO part
          /*************************************************************/
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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

#else

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "spline.h"

int main() {
  std::vector<double> X(5), Y(5);
  X[0] = 0.1; X[1] = 0.4; X[2] = 1.2; X[3] = 1.8; X[4] = 2.0;
  Y[0] = 0.1; Y[1] = 0.7; Y[2] = 0.6; Y[3] = 1.1; Y[4] = 0.9;

  tk::spline s;
  s.set_points(X, Y);

  double x = 1.5;
  printf("spline at %f is %f\n", x, s(x));


  std::cout << "Hello World!" << std::endl;
  return EXIT_SUCCESS;
}

#endif

