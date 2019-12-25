#include <ros/ros.h>
#include <local_interface/SubmitFBMessageApi.h>
#include <local_interface/InstanceId.h>
#include <time.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

static int s_ins_id = -1;

void SubmitFBMessageThread(std::string& fb_message){
    while(ros::ok()){
      HttpRequestParameter http_param;
      http_param.b_has_cookies_ = false;
      http_param.str_url_ = "http://111.231.84.43:8002/monitor/upper/";
      http_param.content_type_ = "application/x-www-form-urlencoded";
      http_param.command_type_ = POST;
      http_param.time_out_ = 300;
      http_param.post_data_ = fb_message;

      SubmitFBMessageApi s_fb_api;
      s_fb_api.Request(http_param);

      boost::this_thread::sleep(boost::posix_time::millisec(300));
    }
}		

void GetInstanceIdCb(const local_interface::InstanceId& msg){
  if(s_ins_id != msg.ins_id){
    s_ins_id = msg.ins_id;
    
    std::cout << s_ins_id << std::endl;

    if(s_ins_id != -1){
      std::cout << "Create thread" << std::endl;
      
      std::string fb_message = "remote control on";
      boost::thread submit_fbmessage(SubmitFBMessageThread, fb_message);
      submit_fbmessage.join();
    }
  }
}

int main(int argv, char** argc){
  ros::init(argv, argc, "SubmitFBMessage");
  ros::NodeHandle n;
  ros::Subscriber instance_id_sub = n.subscribe("InstanceId", 1, GetInstanceIdCb);
  
  std::cout << "in main" << std::endl;

  ros::spin();

  return 0;
}
