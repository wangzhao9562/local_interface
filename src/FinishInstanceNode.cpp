/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  FinishInstanceNode.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/9/1
  * - Brief:     Definition of FinishInstanceNode.
  *****************************************************************************
**/

#include <ros/ros.h>
#include <local_interface/FinishInstanceApi.h>
#include <local_interface/InstanceId.h>
#include <time.h>
#include <iostream>

static int f_ins_id = -1;

void GetInstanceIdCb(const local_interface::InstanceId& msg){
  if(f_ins_id != msg.ins_id){
    f_ins_id = msg.ins_id;

    std::cout << f_ins_id << std::endl;

    time_t f_time = time(NULL);
    asctime(localtime(&f_time));
    FinishInstanceApi f_api(f_ins_id, f_time);

    HttpRequestParameter f_http_param;
    f_http_param.b_has_cookies_ = false;
    f_http_param.str_url_ = "http://111.231.84.43:8002/monitor/upper/";
    f_http_param.content_type_ = "application/x-www-form-urlencoded";
    f_http_param.command_type_ = POST;
    f_http_param.post_data_ = "id=" + std::to_string(f_ins_id) + 
                            "&time=" + std::to_string(f_time);
    f_api.Request(f_http_param);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "FinishInstance");
  ros::NodeHandle n;
  ros::Subscriber instance_id_sub = n.subscribe("InstanceId", 1, GetInstanceIdCb);
  
  ros::spin();

  return 0;
}
