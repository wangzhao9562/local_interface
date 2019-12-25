#include <local_interface/NetManagerROS.h>
#include <algorithm>

NetManagerROS::NetManagerROS(std::string name){
  Initialize(name);
}

void NetManagerROS::Initialize(std::string name){
  ctrl_command_ = "";
  lat_ori_ = 30.0000;
  lng_ori_ = 114.0000;
  is_finish_ = false;

  // ros component intialization
  ros::NodeHandle private_nh("~/" + name);
  ros::NodeHandle nh;

  ins_id_pub_ = nh.advertise<local_interface::InstanceId>("InstanceId", 1);
  ctrl_command_pub_ = nh.advertise<std_msgs::String>("CtrlCommand", 1);

  status_sub_ = nh.subscribe("position", 1, &NetManagerROS::StatusCallback, this);
  finish_msg_sub_ = nh.subscribe("finish_instance", 1, &NetManagerROS::FinishMsgCallback, this);
 
  dsrv_ = new dynamic_reconfigure::Server<local_interface::NetManagerConfig>(private_nh);
  dynamic_reconfigure::Server<local_interface::NetManagerConfig>::CallbackType dsrv_cb = boost::bind(&NetManagerROS::ReconfigureCB, this, _1, _2);
  dsrv_->setCallback(dsrv_cb);

  is_create_ins_ = false;
}


void NetManagerROS::ReconfigureCB(local_interface::NetManagerConfig& config, uint32_t level){
  ship_id_ = config.ship_id;
  base_url_ = config.base_url;
  fb_message_ = config.fb_message;
  is_get_ctrl_ = config.is_get_control;
  setting_speed_ = config.speed;
  setting_heading_ = config.heading;
}

// void NetManagerROS::FinishMsgCallback(const local_interface::FinishMsg::ConstPtr& msg){
//  is_finish_ = msg->is_finish;
// }

void NetManagerROS::StatusCallback(const hust_arms_usv_msgs::NavStatus::ConstPtr& msg){
  ship_id_ = msg->ship_num;
	
  status_.lat_ = msg->global_position.latitude;
  status_.lon_ = msg->global_position.longitude;

  status_.alti_ = msg->global_position.altitude;

  status_.lat_ori_ = static_cast<long double>(msg->origin.latitude);
  status_.lon_ori_ = static_cast<long double>(msg->origin.longitude);

  status_.pos_x_ = msg->related_position.north;
  status_.pos_y_ = msg->related_position.east;

  status_.f_error_ = msg->follow_diff;

  status_.phi_ = msg->yaw;
  status_.gps_phi_ = msg->yaw;

  status_.rud_ = msg->fixed_param.rudder_angle;
  status_.speed_ = msg->fixed_param.velocity;
  status_.gear_ = msg->fixed_param.gear;
  status_.time_ = msg->header.stamp.sec;

  status_.kp_ = msg->control_param.kp;
  status_.ki_ = msg->control_param.ki;
  status_.kd_ = msg->control_param.kd;
  status_.k1_ = msg->control_param.kp1;
  status_.k2_ = msg->control_param.ki1;

  status_.temp_ = msg->sensor_param.temperature;
  status_.ph_ = msg->sensor_param.ph;
  status_.diso_ = msg->sensor_param.desolved_oxygen;
  status_.tur_ = msg->sensor_param.turbidity;
  status_.con_ = msg->sensor_param.conductivity;

  status_.UpdateCheckValue();
}

void NetManagerROS::FinishMsgCallback(const local_interface::FinishMsg::ConstPtr& msg){
  is_finish_ = msg->is_finish;
  if(is_finish_){
    ROS_INFO("local interface: Master request to finish Instance!");
    CallSrv(NetManagerROS::SrvMode::Stop);

    NetFinishInstance(ins_id_); // request to finish instance
  }
}


void NetManagerROS::NetCreateNewInstance(InstanceData ins){
  CreateInstanceApi api(ins);

  HttpRequestParameter http_param;
  http_param.b_has_cookies_ = false;
  http_param.str_url_ = base_url_;
  http_param.content_type_ = "application/x-www-form-urlencoded";
  http_param.command_type_ = POST;
  http_param.post_data_ = "name=" + ins.name_ +
                          "&desp=" + ins.desp_ +
                          "&amount=" + std::to_string(ins.amount_) + 
                          "&shape=" + ins.shape_ +
                          "&time=" + std::to_string(ins.time_);
  http_param.time_out_ = 200;

  api.Request(http_param);

  ins_id_ = api.r_ins_id_;

  is_create_ins_ = true;

  boost::thread ins_id_pub(boost::bind(&NetManagerROS::PublishInstanceId, this, ins_id_));
  ins_id_pub.detach();
}

void NetManagerROS::PublishInstanceId(int instance_id){
    while(ros::ok()){
      if(is_create_ins_){
        local_interface::InstanceId ins_id_msg;
        ins_id_msg.ins_id = ins_id_;
        ins_id_pub_.publish(ins_id_msg);
        ROS_INFO_STREAM("instance id: " << ins_id_);     

        boost::this_thread::sleep(boost::posix_time::millisec(200));
      }
    }
}

void NetManagerROS::NetFinishInstance(int ins_id){
  while(ros::ok()){
    if(is_create_ins_ && is_finish_){
      time_t cur_time = time(NULL);
      asctime(localtime(&cur_time));
      FinishInstance(cur_time, ins_id);
    }
    
    boost::this_thread::sleep(boost::posix_time::millisec(200));
  }
}

void NetManagerROS::FinishInstance(int time, int ins_id){
  FinishInstanceApi api(time, ins_id);

  HttpRequestParameter http_param;
  http_param.b_has_cookies_ = false;
  http_param.str_url_ = base_url_;
  http_param.content_type_ = "application/x-www-form-urlencoded";
  http_param.command_type_ = POST;
  http_param.post_data_ = "id=" + std::to_string(ins_id) +
                          "&time=" + std::to_string(time);

  api.Request(http_param);
}

void NetManagerROS::NetSubmitParam(int ss_id){
  while(ros::ok()){
    if(is_create_ins_){
      ROS_INFO("local_interface: Ready to submit status param");
      SubmitParam(ss_id, status_);
    }

    boost::this_thread::sleep(boost::posix_time::millisec(200));
  }
}

void NetManagerROS::SubmitParam(int ss_id, SShipParam ss_param){
  SubmitParamApi api;

  CSubmitData submit_data;
  submit_data.id_ = ss_id;
  submit_data.param_ = ss_param;  
  HttpRequestParameter http_param;
  http_param.b_has_cookies_ = false;
  http_param.str_url_ = base_url_;
  http_param.content_type_ = "application/x-www-form-urlencoded";
  http_param.command_type_ = POST;
  http_param.post_data_ = SubmitParamApi::GetPostData(submit_data, ins_id_);

  api.Request(http_param);
}

void NetManagerROS::NetSubmitFBMessage(std::string message){
  while(ros::ok()){
    if(is_create_ins_){
      SubmitFBMessage(message);
    }

    boost::this_thread::sleep(boost::posix_time::millisec(200));
  }
}

void NetManagerROS::SubmitFBMessage(std::string message){
  SubmitFBMessageApi api;
 
  HttpRequestParameter http_param;
  http_param.b_has_cookies_ = false;
  http_param.str_url_ = base_url_;
  http_param.content_type_ = "application/x-www-form-urlencoded";
  http_param.command_type_ = POST;
  http_param.time_out_ = 200;
  http_param.post_data_ = message;

  api.Request(http_param);
}

void NetManagerROS::NetGetControlData(int ins_id, bool is_get_control){
  while(ros::ok()){
    if(is_create_ins_){
      GetControlData(ins_id, is_get_control);
    }

    boost::this_thread::sleep(boost::posix_time::millisec(200));
  }
}


void NetManagerROS::GetControlData(int ins_id, bool is_get_control){
    // Create api to get control command  
    GetControlApi api(ins_id);
    api.SetControlStatus(is_get_control);

    // Initialize api 
    HttpRequestParameter http_param;
    http_param.b_has_cookies_ = false;
    http_param.str_url_ = base_url_;
    http_param.content_type_ = "application/x-www-form-urlencoded";
    http_param.command_type_ = GET;
    http_param.time_out_ = 200;

    api.Request(http_param);

    // Publish control command
    std_msgs::String ctrl_command_msg;
    ctrl_command_msg.data = api.get_command_;
    ctrl_command_ = api.get_command_;
    ctrl_command_pub_.publish(ctrl_command_msg);

    // Unpack the control command
    CtrlCommandSolver ctrl_solver(ctrl_command_, lat_ori_, lng_ori_);

    std::vector<std::string> split_str_vec;
    CtrlCommandSolver::CtrlMode control_mode = ctrl_solver.GetCtrlMode('&', split_str_vec);

    // Execute mission
    if(control_mode == CtrlCommandSolver::CtrlMode::MultiPointFollow){
       std::vector<std::pair<double, double>> temp_vec;
       int follow_count;
       if((ctrl_solver.GetWayPoints(split_str_vec, temp_vec, follow_count)) && follow_count_ != follow_count){
         /* Execution of multi point following */ 
       }
    }

    if(control_mode  == CtrlCommandSolver::CtrlMode::PointFollow){
      long double lat, lng; 
      if(ctrl_solver.GetTarget(split_str_vec, lat, lng)){
        if(CallSrv(NetManagerROS::SrvMode::PointFollow, lat, lng)){
          while(!is_reached_){
            ROS_INFO("local_interface: Has not reached the goal"); 
          }
          CallSrv(NetManagerROS::SrvMode::StationKeep, lat, lng);
        }
        else{
          CallSrv(NetManagerROS::SrvMode::Stop);
        } 
      }
    }
    if(control_mode == CtrlCommandSolver::CtrlMode::Stop){
      if(CallSrv(NetManagerROS::SrvMode::Stop)){
        ROS_INFO("local_interface: Stop successfully!");
      }
      else{
        ROS_ERROR("local_interface: Error to stop!");
      }
    }
}


void NetManagerROS::Action(){
  boost::thread(boost::bind(&NetManagerROS::NetSubmitParam, this, 1)).detach();
  boost::thread(boost::bind(&NetManagerROS::NetSubmitFBMessage, this, fb_message_)).detach();
  boost::thread(boost::bind(&NetManagerROS::NetGetControlData, this, ins_id_, is_get_ctrl_)).detach();
}

void NetManagerROS::UpdateStatus(SShipParam ss_param){
  status_ = ss_param;
}

int NetManagerROS::CallSrv(NetManagerROS::SrvMode srv_mode, double lat, double lng){
   if(srv_mode == NetManagerROS::SrvMode::PointFollow){
       /* call point follow service */
       bool is_call_srv = true; // default
       return static_cast<int>(is_call_srv);
    }
    if (srv_mode == NetManagerROS::SrvMode::Stop){
       /* call stop service */
       bool is_call_srv = true; // default
       return static_cast<int>(is_call_srv);
    }
    if(srv_mode == NetManagerROS::SrvMode::StationKeep){
       /* call station keep service */ 
       bool is_call_srv = true; // default
       return static_cast<int>(is_call_srv);
   }
   return -1;
 }
