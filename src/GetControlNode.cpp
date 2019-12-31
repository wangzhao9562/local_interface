/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  GetControlNode.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/9/1
  * - Brief:     ROS node to test GetControlApi
  *****************************************************************************
**/

#include <ros/ros.h>
#include <local_interface/GetControlApi.h>
#include <local_interface/InstanceId.h>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

static int g_ins_id = -1;


void GetControlThread(int instance_id){
  while(ros::ok()){
    GetControlApi c_api(instance_id);
    c_api.SetControlStatus(true);

    HttpRequestParameter http_param;
    http_param.b_has_cookies_ = false;
    http_param.str_url_ = "http://111.231.84.43:8002/monitor/upper/";
    http_param.content_type_ = "application/x-www-form-urlencoded";
    http_param.command_type_ = GET;
    http_param.time_out_ = 300;
  
    c_api.Request(http_param);

    boost::this_thread::sleep(boost::posix_time::millisec(300));
  }
}


void GetInstanceIdCb(const local_interface::InstanceId& msg){
  if(g_ins_id != msg.ins_id){
    g_ins_id = msg.ins_id;
  }

  if(g_ins_id != -1){
    boost::thread get_control_data(GetControlThread, g_ins_id); 
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "GetControl");
  ros::NodeHandle n;
  ros::Subscriber ins_id_sub = n.subscribe("InstanceId", 1, GetInstanceIdCb);

  if(g_ins_id != -1){
    boost::thread get_control_data(GetControlThread, g_ins_id); 
  }

  ros::spin();
    
  return 0;
}
