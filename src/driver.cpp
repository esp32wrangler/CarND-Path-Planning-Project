//
//  driver.cpp
//  Path_Planning
//
//  Created by Tamas Kerecsen on 2020. 05. 06..
//

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "driver.hpp"
#include "spline.h"
#include <iostream>
#include <fstream>


Driver::Driver(const char*  mapfilename)
{
  loadMap(mapfilename);
  lane = 1;
  ref_speed = 22; //m/s
  max_delta_v_per_frame = simulator_frame_time*9;
  target_speed = ref_speed;
  speed_urgency = 0.5;
  current_speed = 0;
  following_distance = 20;
}

void Driver::loadMap(const char* fname)
{
  // Load up map values for waypoint's x,y,s and d normalized normal vectors

  
    // Waypoint map to read from
    string map_file_ = fname;
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
  
}



std::pair<std::vector<double>, std::vector<double>> Driver::getTrajectory(const nlohmann::json& car_data)
{
  
   // Main car's localization Data
  double car_x = car_data["x"];
  double car_y = car_data["y"];
  double car_s = car_data["s"];
  double car_d = car_data["d"];
  double car_yaw = car_data["yaw"];
  double car_speed = car_data["speed"];

  // Previous path data given to the Planner
  auto previous_path_x = car_data["previous_path_x"];
  auto previous_path_y = car_data["previous_path_y"];
  int previous_path_size = previous_path_x.size();
  // Previous path's end s and d values
  double end_path_s = car_data["end_path_s"];
  double end_path_d = car_data["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side
  //   of the road.
  auto sensor_fusion = car_data["sensor_fusion"];
 
  if (previous_path_size > 0)
  {
    car_s = end_path_s;
  }
  
  bool car_in_front = false;
  double car_in_front_distance = -1;
  
  vector<double>lane_speeds(3, ref_speed);
  vector<bool>lane_available(3, true);
  vector<bool>lane_marginally_available(3, true);
  vector<vector<double>>carpos(3);
  
  double avoidance_speed = 0;
  double avoidance_urgency = 0;


  for (int i = 0; i < sensor_fusion.size(); ++i)
  {
    double other_d = sensor_fusion[i][6];
  }
  
  for (int i = 0; i < sensor_fusion.size(); ++i)
  {
    double other_d = sensor_fusion[i][6];
    if (other_d < 0)
    {
      continue;
    }
    int other_lane = floor(other_d/4.0);
    double other_vx = sensor_fusion[i][3];
    double other_vy = sensor_fusion[i][4];
    double other_speed = sqrt(other_vx*other_vx+other_vy*other_vy);
    double other_s = sensor_fusion[i][5];
    // project out the other car's position to where it will be at the end of our laid down path
    other_s += previous_path_size*simulator_frame_time*other_speed;

    double dist_to_other = other_s-car_s;
    // if we are near the rollover, and another car in front of us has already rolled over, distance is sum of the distance of the two cars from the rollover point
    if (dist_to_other < -6000)
    {
      dist_to_other = other_s + 6945.56-car_s;
    }
    if (dist_to_other > 6000)
    {
      dist_to_other = car_s + 6945.56-other_s;
    }

//    std::cout << other_lane << ", " << std::setw(10) << other_d << " " << std::setw(10) << other_s << " " << std::setw(10) << other_speed << " " << std::setw(10) << dist_to_other << std::endl;

//    std::cout << "other s" << other_s << " other speed " << other_speed << " distance " << other_s - car_s << std::endl;

    carpos[other_lane].push_back(dist_to_other);
    if (dist_to_other > -10) {
      if (dist_to_other < 60) // if there is a slow car just beyond the 30 m limit, give a chance to pass the car in front and get back to our lane
      {
        if (other_speed < lane_speeds[other_lane])
        {
          lane_speeds[other_lane] = other_speed;
        }
      }
      if (dist_to_other < 30)
      {
        lane_available[other_lane] = false;
      }
    }
    if (dist_to_other > -5  && dist_to_other < 15 )
    {
      lane_marginally_available[other_lane] = false;
    }
    
    double dangerzone_min = 2 + 4*lane - 2;
    double dangerzone_max = 2 + 4*lane + 2;
    // deal with the guys who are changing lanes in front of us
    if (lane > 0)
    {
      dangerzone_min -= 1;
    }
    if (lane < 2)
    {
      dangerzone_max += 1;
    }

    if (other_d < dangerzone_max && other_d > dangerzone_min) {
      if (other_s > car_s && dist_to_other < 30)
      {
        if (!car_in_front || dist_to_other < car_in_front_distance)
        {
          // collision danger
          avoidance_speed = other_speed;
          if (dist_to_other < following_distance)
          {
            avoidance_speed -= 1;
            //std::cout << " too close ";
          }
          double old_required_deceleration = -1* (other_speed*other_speed - current_speed*current_speed)/(2*(other_s-car_s));
          double distance_to_break = (dist_to_other > following_distance) ? dist_to_other-following_distance : 1;
          double required_deceleration = pow(current_speed-other_speed, 2) / (2*distance_to_break);
          if (required_deceleration > 5)
          {
            distance_to_break = (dist_to_other > following_distance/2) ? dist_to_other-(following_distance/2) : 1;
            required_deceleration = pow(current_speed-other_speed, 2) / (2*distance_to_break);
            //std::cout << "extended braking " << std::endl;
          }
          if (required_deceleration > 9)
          {
            distance_to_break = (dist_to_other > following_distance/10) ? dist_to_other-(following_distance/10) : 1;
            required_deceleration = pow(current_speed-other_speed, 2) / (2*distance_to_break);
            //std::cout << "extended braking2 " << std::endl;
          }
          avoidance_urgency = (required_deceleration*simulator_frame_time)/max_delta_v_per_frame;
          if (dist_to_other < following_distance && avoidance_urgency < 0.3)
          {
            avoidance_urgency += 0.3;
            //std::cout << " too close ";
          }
          car_in_front = true;
          car_in_front_distance = dist_to_other;
          //std::cout << "ur " << avoidance_urgency << " sp " << current_speed << " osp " << other_speed << " dist " << dist_to_other << " db " << distance_to_break << " a " << required_deceleration << "(" << old_required_deceleration << " l " << lane << std::endl;
          if (speed_urgency > 1)
          {
            //std::cout << " about to crash "; // TODO: deal with this situation
          }
        }
      }
    }
  }
  
  /*std::cout << "Lane speeds: ";
  for (int i : {0, 1, 2})
  {
    std::cout << lane_speeds[i] << " (" << lane_available[i] << ") " ;
  }
  std::cout << std::endl;
  
  for (int i : {0, 1, 2})
  {
    std::cout << "Lane " << i << ": ";
    std::sort(carpos[i].begin(), carpos[i].end(), std::greater<int>());
    for (auto &item : carpos[i]) {
      std::cout << item << ", ";
    }
    std::cout << std::endl;
  }*/
  
  if (car_in_front)
  {
    bool lane_change = false;
    for (int alt_lane : {lane+1, lane-1})
    {
      float best_speed = avoidance_speed;
      int best_lane = lane;
      if (alt_lane < 3 and alt_lane >= 0)
      {
        if (lane_speeds[alt_lane] > best_speed && (lane_available[alt_lane] || (lane_marginally_available[alt_lane] && avoidance_urgency > 0.9)))
        {
          best_speed = lane_speeds[alt_lane];
          best_lane = alt_lane;
          if (avoidance_urgency > 0.9)
          {
            //std::cout << "marginal lane " << alt_lane << std::endl;
          }
        }
      }
      if (best_lane != lane)
      {
          lane = alt_lane;
          lane_change = true;
      }
    }
    if (!lane_change)
    {
      target_speed = avoidance_speed;
      speed_urgency = avoidance_urgency;
      if (avoidance_urgency > 1)
      {
        avoidance_urgency = 1; // hope for the best
      }
    }
  }
  else
  {
    target_speed = ref_speed;
    speed_urgency = 0.5;
  }
//  std::cout << std::endl;
  
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  double ref_prev_x = 0;
  double ref_prev_y = 0;
  vector<double> spaced_points_x, spaced_points_y;

  if (previous_path_size < 2)
  {
    // if running out of points (or at startup), use a virtual point 1 meter behind the car in the direction of the yaw
    // to define a starting vector for the new plan
    ref_prev_x = car_x - cos(car_yaw);
    ref_prev_y = car_y - sin(car_yaw);
  }
  else
  {
    // otherwise take the last two points of the previously planned path
    ref_x = previous_path_x[previous_path_size-1];
    ref_y = previous_path_y[previous_path_size-1];
    
    ref_prev_x = previous_path_x[previous_path_size-2];
    ref_prev_y = previous_path_y[previous_path_size-2];
    
    ref_yaw = atan2(ref_y-ref_prev_y, ref_x-ref_prev_x);
    
  }
  spaced_points_x.push_back(ref_prev_x);
  spaced_points_x.push_back(ref_x);
  spaced_points_y.push_back(ref_prev_y);
  spaced_points_y.push_back(ref_y);

  double next_s = car_s;
  int spacing = 30;
  for (int i = 0; i < 3; ++i)
  {
    next_s += spacing;
    double next_d = 2+4*lane;
    auto xy = getXY (next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    spaced_points_x.push_back(xy[0]);
    spaced_points_y.push_back(xy[1]);
  }
  

  for (int i = 0; i < spaced_points_x.size(); ++i)
  {
    double shift_x = spaced_points_x[i] - ref_x;
    double shift_y = spaced_points_y[i] - ref_y;
    
    spaced_points_x[i] = shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
    spaced_points_y[i] = shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
  }
  
  tk:: spline spl;
  
  spl.set_points(spaced_points_x, spaced_points_y);
  
  
  auto trajectory = make_pair(std::vector<double>(), std::vector<double>());

  for (int i = 0; i < previous_path_size; ++i)
  {
    trajectory.first.push_back(previous_path_x[i]);
    trajectory.second.push_back(previous_path_y[i]);
  }
  
  // figure out the step on a linearized section of the spline to reach desired speed
  double target_x = spacing;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x*target_x+target_y*target_y);
  
  double next_x_on_spline = 0;
  
  for (int i = 0; i < 50-previous_path_size; ++i) {
    if (target_speed != current_speed)
    {
      double diff = target_speed-current_speed;
      double delta = max_delta_v_per_frame*speed_urgency;
      if (abs(diff) < delta)
      {
        delta = diff;
      }
      else
      {
        delta = copysign(delta, diff);
      }
      current_speed += delta;
    }
    double N = target_dist/(simulator_frame_time*current_speed);
    double dist_inc = target_x/N;

    
    next_x_on_spline += dist_inc;
    double next_y_on_spline = spl(next_x_on_spline);

    double x_on_map = next_x_on_spline*cos(ref_yaw)-next_y_on_spline*sin(ref_yaw) + ref_x;
    double y_on_map = next_x_on_spline*sin(ref_yaw)+next_y_on_spline*cos(ref_yaw) + ref_y;
    
    trajectory.first.push_back(x_on_map);
    trajectory.second.push_back(y_on_map);

  }

  return trajectory;
}
