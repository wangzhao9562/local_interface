#include <ros/ros.h>
#include <local_interface/CreateInstanceApi.h>
#include <local_interface/InstanceId.h>
#include <time.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "CreateInstance");
  ros::NodeHandle n;
  ros::Publisher instance_id_pub = n.advertise<local_interface::InstanceId>("InstanceId", 10);
  ros::Rate loop_rate(10);

  time_t cur_time = time(NULL);
  asctime(localtime(&cur_time));
  InstanceData instance("Local test", "LH", 3, "ABC", cur_time);

  HttpRequestParameter http_param;
  http_param.b_has_cookies_ = false;
  http_param.str_url_ = "http://111.231.84.43:8002/monitor/upper/";
  http_param.content_type_ = "application/x-www-form-urlencoded";
  http_param.command_type_ = POST;
  http_param.post_data_ = "name=" + instance.name_ + 
                          "&desp=" + instance.desp_ +
                          "&amount=" + std::to_string(instance.amount_) +
                          "&shape=" + instance.shape_ +
                          "&time=" + std::to_string(instance.time_);
  http_param.time_out_ = 300;

  CreateInstanceApi c_api(instance);
  c_api.Request(http_param);

  while(ros::ok()){
    local_interface::InstanceId ins_id_msg;
    ins_id_msg.ins_id = c_api.r_ins_id_;

    instance_id_pub.publish(ins_id_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
