#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

void printPts(vector<double> &ptsx, vector<double> &ptsy)
{
  printf("x,    y\n");
  for (int i = 0; i < ptsx.size(); i++)
  {
    printf("%f,  %f\n", ptsx[i], ptsy[i]);
  }
}

void printPts(vector<double> &pts)
{
  printf("Pts(%ld):", pts.size());
  for (auto p : pts)
    printf("%f ", p);
  printf("\n");
}

int main()
{
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
  while (getline(in_map_, line))
  {
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

    //printf("Loop\n");
  }

  int my_lane = 1;       // 0 is left edge
  double ref_vel = 0; // reference velocity [mph]

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &my_lane, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
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

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_size = previous_path_x.size();
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          // prediction
          bool too_close = false;
          bool isExistRight = false;
          bool isExistLeft = false;
          std::cout << std::fixed;
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            // car is my lane
            float d = sensor_fusion[i][6];
            int sensing_car_lane = 0;
            //if (d < (2 + 4 * my_lane) && d > (2 + 4 * my_lane - 2))
            if(0 <= d && d < 4)
            {
              // sensing car is my lane
              sensing_car_lane = 0;
            }
            else if(4 <= d && d < 8)
            //else if (d < (2 + 4 * (my_lane - 1)) && d > (2 + 4 * (my_lane - 1) - 2))
            {
              // sensing car is left
              sensing_car_lane = 1;
            }
            else if(8 <= d && d <= 12)
            //else if (d < (2 + 4 * ((my_lane + 1) - 1)) && d > (2 + 4 * ((my_lane + 1) - 1) - 2))
            {
              // sensing car is right
              sensing_car_lane = 2;
            }
            else
            {
              continue;
            }

            double check_car_s = sensor_fusion[i][5];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_d = sensor_fusion[i][6];

            // if using previous points can preject s value out
            check_car_s += ((double)prev_size * .02 * check_speed);

            if (sensing_car_lane == my_lane)
            {
              too_close |= (check_car_s > car_s) && ((check_car_s - car_s) < 30);
              if ((check_car_s - car_s) > 0)
                std::cout << "            " << i << ":" << std::setprecision(2) << (check_car_s - car_s) << std::endl;
            }
            if (sensing_car_lane == my_lane - 1)
            {
              isExistLeft |= ((car_s - 15) < check_car_s) && ((car_s + 30) > check_car_s);
              if ((check_car_s - car_s) > 0)
                std::cout << i << ":" << std::setprecision(2) << (check_car_s - car_s) << std::endl;
            }
            if (sensing_car_lane == my_lane + 1)
            {
              isExistRight |= ((car_s - 15) < check_car_s) && ((car_s + 30) > check_car_s);
              if ((check_car_s - car_s) > 0)
                std::cout << "                    " << i << ":" << std::setprecision(2) << (check_car_s - car_s) << std::endl;
            }
          }
          // select behavior
          printf("Sensing:%d %d %d   Mylane:%d  State:", isExistLeft ? 1 : 0, too_close ? 1 : 0, isExistRight ? 1 : 0, my_lane);
          if (too_close)
          {
            if ((my_lane > 0) && (!isExistLeft))
            {
              my_lane -= 1;
              printf("Left ");
            }
            else if ((my_lane < 2) && (!isExistRight))
            {
              my_lane += 1;
              printf("Right ");
            }
            else
            {
              printf("Deaccerate ");
              ref_vel -= .224;
            }

          }
          else
          {
            if (my_lane == 0 && !isExistRight || my_lane == 2 && !isExistLeft)
            {
                printf("Back center ");
                my_lane = 1;
            }
//            else if ((my_lane == 2) && (isExistLeft))
//            {
//              printf("Left ");
//              my_lane = 1;
//            }

            if (ref_vel < 49.5)
            {
              printf("Accerate ");
              ref_vel += .224;
            }
            else
            {
              printf("Keep ");

            }
          }

          printf("\n");

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          //printf("\nprevious size:%d\n",prev_size);
          //printPts(ptsx, ptsy);
          if (prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

            //printPts(ptsx,ptsy);
          }
          else
          {
            // 角度を計算している,基準座標を作成する準備
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * my_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * my_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * my_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          //printPts(ptsx, ptsy);
          // To local car coordinates.
          for (int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // spline
          tk::spline s;
          //printPts(ptsx,ptsy);
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // スプラインに聞くんです。xはこれなのでyは何ですかと。
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
          //printf("target Y:%f\n",target_y);

          double x_add_on = 0;

          for (int i = 1; i < 50 - prev_size; i++)
          {

            double N = target_dist / (0.02 * ref_vel / 2.24); // ターゲットまでの到達時間
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            //printf("はいってますか？目標は%f\n", ref_vel);
          }

          //printPts(next_x_vals,next_y_vals);

          msgJson["next_x"] = next_x_vals; // ここを変えないと動かない
          msgJson["next_y"] = next_y_vals; // ここを変えないと動かない

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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