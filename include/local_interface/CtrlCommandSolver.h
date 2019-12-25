#pragma once
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>

#define PI 3.1415926

/* @TODO: Analysis received control command
 */
class CtrlCommandSolver{

public:
  /* @brief Enum class to list control mode
   */
  enum class CtrlMode{
    Stop,
    OpenControl,
    PointFollow,
    GeneralLineFollow,
    SpecialLineFollow,
    MultiLineFollow,
    CircleFollow,
    MultiPointFollow,
    Format,
    ExpectCtrl
  };

public:
  /* @brief Constructor with input
   * @param command Input control command  
   */
  CtrlCommandSolver(std::string command, long double lat_ori, long double lng_ori) : command_(command), lat_ori_(lat_ori), lng_ori_(lng_ori) {}

  /* @brief Test interface
   * @param split_char Split mark in control command
   */
  void PrintTest(const char split_char);

  /* @brief Get the control mode from control command
   * @param split_char The symbol to split command string
   * @param split_str Vector to store the result of split
   */
  CtrlCommandSolver::CtrlMode GetCtrlMode(const char split_char, std::vector<std::string>& split_str);

  /* @brief In point following controlling mode, get the target point from command 
   * @param str_vec The result of split 
   * @param lat Store the latitude of target point
   * @param lng Store the longitude of target point
   * @return Return if the process is correct
   */
  bool GetTarget(std::vector<std::string>& str_vec, long double& lat, long double& lng);	

  /* @brief In multi point following controlling mode, get all waypoints from command
   * @param str_vec The result of split
   * @param waypoints Store all waypoints from commands
   * @param follow_count The times of path following called
   * @return Return if the process is correct
   */
  bool GetWayPoints(std::vector<std::string>& str_vec, std::vector<std::pair<double, double>>& waypoints, int& follow_count);

private:
  /* @brief Split input control command according ot offered split char
   * @param split_char Offered char to split the control command
   * @return A vector of string after split the control command
   */
  std::vector<std::string> Split(const char split_char);

  /* @brief Analysis the split result, and return the control mode in command
   * @param split_char Offered char to split the control command
   * @param split_str Vector of string after split the control command
   * @return The control mode in command
   */
  CtrlCommandSolver::CtrlMode Analysis(const char split_char, std::vector<std::string>& split_str);

  /* @brief Transform point in latlng to xy
   * @param pos_x Input x position
   * @param pos_y Input y position
   * @return Return the latitude and longitude in pair format
   */
  std::pair<double, double> XYToLatLng(double pos_x, double pos_y);
  
private:
  std::string command_; // contrl command
  
  double lat_ori_; // latitude of original point 
  double lng_ori_; // longitude of original point

  const double earth_e_ = 0.03352810664;
  const double earth_a_ = 6378137.0;
};

