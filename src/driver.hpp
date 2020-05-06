//
//  driver.hpp
//  Path_Planning
//
//  Created by Tamas Kerecsen on 2020. 05. 06..
//

#ifndef driver_hpp
#define driver_hpp

#include <iostream>
#include <vector>
#include "json.hpp"
#include "spline.h"

class Driver
{
protected:
  void loadMap(const char* filename);
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
  tk::spline left_lane_spline;
  tk::spline center_lane_spline;
  tk::spline right_lane_spline;
  int lane;
  double ref_speed;
  double max_delta_v_per_frame;
  double simulator_frame_time = 0.02;
  double target_speed;
  double current_speed;
  double speed_urgency;
  
  void prepareSplines();

  std::pair<double, double> getXYwithSpline(double s, double d);
  
public:
  Driver(const char* mapfilename);
  std::pair<std::vector<double>, std::vector<double>> getTrajectory(const nlohmann::json& car_data);
};

#endif /* driver_hpp */
