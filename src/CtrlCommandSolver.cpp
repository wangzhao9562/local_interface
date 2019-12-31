/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  CtrlCommandSolver.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/9/1
  * - Brief:     Definition of CtrlCommandSolver.
  *****************************************************************************
**/

#include <local_interface/CtrlCommandSolver.h>
#include <iostream>
#include <sstream>

void CtrlCommandSolver::PrintTest(const char split_char){
  std::vector<std::string> split_strs;
  CtrlCommandSolver::CtrlMode mode = Analysis(split_char, split_strs);
  
  if(mode == CtrlCommandSolver::CtrlCommandSolver::CtrlMode::PointFollow){
    std::cout << "Enter print process" << std::endl;
    auto lat_str = split_strs[4];
    auto lon_str = split_strs[5];

    std::cout << "lat:" << lat_str << " lon:" << lon_str << std::endl;
  }
  if(mode == CtrlCommandSolver::CtrlMode::Stop){
    std::cout << "Stop" << std::endl;
  }
}

CtrlCommandSolver::CtrlMode CtrlCommandSolver::GetCtrlMode(const char split_char, std::vector<std::string>& split_strs){
  CtrlCommandSolver::CtrlMode mode = Analysis(split_char, split_strs);
  
  return mode;    
}

bool CtrlCommandSolver::GetTarget(std::vector<std::string>& str_vec, long double& lat, long double& lng){
  std::string x_str, y_str;

  try{
    x_str = str_vec[4];
    y_str = str_vec[5];
  }
  catch(std::exception& e){
    std::cout << "Illegal input string vector!" << std::endl;
    return false;
  }

  double pos_x = std::stod(x_str);
  double pos_y = std::stod(y_str);

  // std::pair<double, double> latlng_point = XYToLatLng(pos_x, pos_y);
  // lat = latlng_point.first;
  // lng = latlng_point.second;

  lat = pos_x;
  lng = pos_y;

  return true;
}

bool CtrlCommandSolver::GetWayPoints(std::vector<std::string>& str_vec, std::vector<std::pair<double, double>>& waypoints, int& follow_count){
  follow_count = stoi(str_vec[3]);
  int wp_amount = stoi(str_vec[4]);
  int lat_pos = 5, lng_pos = 6; 

  try{
    for(int i = 0; i < wp_amount; ++i){
      std::stringstream str_stream_lat(str_vec[lat_pos]);
      std::stringstream str_stream_lng(str_vec[lng_pos]);
      double temp_lat, temp_lng;
      str_stream_lat >> temp_lat;
      str_stream_lng >> temp_lng;
      std::pair<double, double> point_latlng(temp_lat, temp_lng);
      waypoints.push_back(point_latlng);
      lat_pos += 2;
      lng_pos += 2;
    }
  }
  catch(std::exception& e){
    std::cout << "Illegal input, error in string split!" << std::endl;
    return false;
  }

  return true;
}

CtrlCommandSolver::CtrlMode CtrlCommandSolver::Analysis(const char split_char, std::vector<std::string>& split_strs){
  split_strs = Split(split_char);

  if(split_strs[0] == "o"){
    return CtrlCommandSolver::CtrlMode::OpenControl;
  }
  if(split_strs[0] == "s"){
    return CtrlCommandSolver::CtrlMode::Stop;
  }
  if(split_strs[0] == "c"){
    if(split_strs[2] == "p"){
      return CtrlCommandSolver::CtrlMode::PointFollow;
    }
    if(split_strs[2] == "g"){
      return CtrlCommandSolver::CtrlMode::GeneralLineFollow;
    }
    if(split_strs[2] == "l"){
      return CtrlCommandSolver::CtrlMode::SpecialLineFollow;
    }
    if(split_strs[2] == "r"){
      return CtrlCommandSolver::CtrlMode::CircleFollow;
    }
    if(split_strs[2] == "m"){
      return CtrlCommandSolver::CtrlMode::MultiPointFollow;
    }
  }

  return CtrlCommandSolver::CtrlMode::ExpectCtrl; 
}

std::vector<std::string> CtrlCommandSolver::Split(const char split_char){
  std::vector<std::string> split_strings;

  auto last_pos = 0;

  int i = 0;
  for(; i < command_.length(); i++){
    if(command_[i] == split_char){
      split_strings.push_back(command_.substr(last_pos, i - last_pos));
      last_pos = i + 1;
    }
  }
  split_strings.push_back(command_.substr(last_pos, command_.length()));

  return split_strings;
}

std::pair<double, double> CtrlCommandSolver::XYToLatLng(double pos_x, double pos_y){
  double d_lat = pos_x * sqrt(pow(1 - pow(earth_e_, 2) * pow(sin(lat_ori_), 2), 3)) * 180 / (earth_a_ * (1 - pow(earth_e_, 2)) * PI);
  double d_lng = -pos_y * sqrt(1 - pow(earth_e_, 2) * pow(sin(lat_ori_), 2) * 180 / earth_a_ * cos(lat_ori_) * PI);
  std::pair<double, double> latlng_pos;
  latlng_pos.first = lat_ori_ + d_lat;
  latlng_pos.second = lng_ori_ + d_lng;

  return latlng_pos;
}
