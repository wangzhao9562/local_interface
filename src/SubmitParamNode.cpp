/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  SubmitParamNode.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/9/1
  * - Brief:     Node to test SubmitParamApi.
  *****************************************************************************
**/

#include <ros/ros.h>
#include <local_interface/SubmitParamApi.h>
#include <local_interface/InstanceId.h>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

static int instance_id = -1;

void SubmitParamThread(int ins_id){
  while(ros::ok()){   
    CSubmitData sb_data;
    sb_data.param_.lat_ = 30.51581833;
    sb_data.param_.lon_ = 114.42678033;
    sb_data.id_ = 1;

    sb_data.param_.UpdateCheckValue();

    HttpRequestParameter http_param;
    http_param.b_has_cookies_ = false;
    http_param.str_url_ = "http://111.231.84.43:8002/monitor/upper/";
    http_param.content_type_ = "application/x-www-form-urlencoded";
    http_param.command_type_ = POST;
    http_param.time_out_ = 300;
    http_param.post_data_ = SubmitParamApi::GetPostData(sb_data, ins_id);

    SubmitParamApi sb_param_api;

    sb_param_api.Request(http_param);

    boost::this_thread::sleep(boost::posix_time::millisec(200));
  }
}

void GetInstanceIdCb(const local_interface::InstanceId& msg){
  if(instance_id != msg.ins_id){
    instance_id = msg.ins_id;
    std::cout << instance_id << std::endl;    
    boost::thread sub_param_thread(SubmitParamThread, instance_id);
    sub_param_thread.join();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "SubmitParam");  
  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  ros::Subscriber ins_id_sub = n.subscribe("InstanceId", 1, GetInstanceIdCb); 

  ros::spin(); 

  return 0;
}
