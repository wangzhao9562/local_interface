/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  NetManagerROS.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/8/1
  * - Brief:     Definition of NetManagerROS, ros wrapper class of NetManager.
  *****************************************************************************
**/

#include <ros/ros.h>
#include <local_interface/InstanceId.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <local_interface/NetManager.h>
#include <local_interface/NetManagerConfig.h>
#include <local_interface/CtrlCommandSolver.h>
#include <dynamic_reconfigure/server.h>
#include <local_interface/FinishMsg.h>
#include <hust_arms_usv_msgs/NavStatus.h>


/* TODO: ROS wrapper class of NetManager, offer ros interface for outside caller
 */
class NetManagerROS{
  friend CSingletonMT<NetManagerROS>;
  friend CSingletonST<NetManagerROS>;

public:
  /* @brief Constructor
   */
  NetManagerROS(std::string name);

  /* @brief Deconstructor
   */
  ~NetManagerROS(){
     if(dsrv_ != nullptr) 
       delete dsrv_;
   };

  /* @brief Initialize
   */
  void Initialize(std::string name);

  /* @brief Call CreateNewInstanceApi to request an instance in server and publish the id
   * @param ins Data of the created instance
   */ 
  void NetCreateNewInstance(InstanceData ins);

  /* @brief Publish ID of the instance
   * @param instance_id ID of the created instance in server
   */
  void PublishInstanceId(int instance_id);

  /* @brief Call FinishInstanceApi to request to finish the instance in server
   * @param ins_id Id of the instance to finish
   */
  void NetFinishInstance(int ins_id);

  /* @brief Call FinishInstanceApi in thread to request to finish the instance in server
   * @param time Time to request to finish the instance
   * @param ins_id Id of the instance to finish
   */
  void FinishInstance(int time, int ins_id);

  /* @brief Call SubmitParamApi to request submit ship/vehicle param to server
   * @param ss_id Id of the ship or vehicle 
   * @param ss_param SShipParam type object which store the param of ship/vehicle
   */
  void SubmitParam(int ss_id, SShipParam ss_param);


  /* @breif Call SubmitParam in thread to request to handle the parameter to server 
   * @param ss_id ID of ship/vehicle
   */
  void NetSubmitParam(int ss_id);

  /* @brief Call SubmitFBMessageApi to request submit fb message to server
   */
  void NetSubmitFBMessage(std::string message);

  /* @brief Call in thread to request submit fb message to server
   * @param message The fb message
   */
  void SubmitFBMessage(std::string message);

  /* @brief Call GetControlApi to request control command from server
   * @param ins_id Id of instance
   * @param is_get_ctrl Is get the control command
   */
  void NetGetControlData(int ins_id, bool is_get_ctrl);

  /* @brief Call in thread to request control command message from server
   * @param ins_id Id of instance
   * @param is_get_ctrl Is get the control command
   */
  void GetControlData(int ins_id, bool is_get_ctrl);
 

  /* @breif Update status parameter of ship or vehicle
   * @param ss_param Latest status
   */
  void UpdateStatus(SShipParam ss_param);

  // /* @brief Call multi threads in parallel way
  //  */
  void Action();

  /* @brief Return instance ID
   * @return ID of the created instance
   */
  int GetInstanceID()const{
    return ins_id_;
  }
  
  /* @brief Return base url
   * @return Base url
   */
  std::string GetBaseUrl()const{
    return base_url_;
  }

  /* @brief Return the fb message
   * @return fb message to submit
   */
  std::string GetFBMessage()const{
    return fb_message_;
  }
 
  enum class SrvMode{
    PointFollow,
    StationKeep,
    Stop
  };

private:
  void ReconfigureCB(local_interface::NetManagerConfig& config, uint32_t level);


  /* @brief Callback function when receive the navigation status message of ship or vehicle 
   * @param msg Navigating status message of HUST ARMs usv
   */
  void StatusCallback(const hust_arms_usv_msgs::NavStatus::ConstPtr& msg);

  /* @brief Callback function when receive the status message of sensor
   * @param msg sensor message
   */
  // void SensorStatusCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    
  /* @brief Callback function when receive the finish message from master monitoring station
   * @param msg Input FinishMsg message
   */
  void FinishMsgCallback(const local_interface::FinishMsg::ConstPtr& msg);

  /* @brief Callback function when receive the goal status message from vehicle
   * @param msg Input GoalStatusArray type message
   */
  // void PointFollowStatusCallback(const actionlib_msgs::GoalStatusArray& msg);
  
  /* @breif Call service to realize stop
   * @param srv_node ROS Service
   * @return Received status of vehicle 
   */
   int CallSrv(NetManagerROS::SrvMode srv_mode);

   /* @breif Call service to realize point following and status keeping
    * @param srv_node ROS service
    * @param lat Input latitude
    * @param lng Input longitude
    * @param Received status of vehicle
    */
   int CallSrv(NetManagerROS::SrvMode srv_mode, double lat, double lng);

private:
  int ship_id_; // Ship or Vehicle ID
  bool is_finish_;
  int ins_id_; // Instance ID

  std::string base_url_; // Base Url
  std::string fb_message_; 
  bool is_get_ctrl_;
  double setting_speed_; 
  double setting_heading_;

  long double lat_ori_;
  long double lng_ori_;

  bool is_reached_ = false;

  ros::Publisher ins_id_pub_; // pipline to observe id of instance
  ros::Publisher ctrl_command_pub_; // pipline to observe control command
  ros::Subscriber status_sub_; 
  ros::Subscriber finish_msg_sub_;
  ros::Subscriber follow_status_sub_; // subscribe realative topic to get status of vehicle 

  /* Preserved for point follow service client */ 
  /* Preserved for stop service client */
  /* Preserved for station keep service client */

  std::string ctrl_command_;
  bool is_create_ins_; // Is instance created 

  SShipParam status_;

  int follow_count_ = 0;; // path follow num account

  dynamic_reconfigure::Server<local_interface::NetManagerConfig>* dsrv_;
}; 
