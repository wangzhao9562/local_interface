#include <local_interface/NetManager.h>

void NetManager::NetCreateNewInstance(InstanceData ins){
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
  http_param.time_out_ = 300;

  api.Request(http_param);
  ins_id_ = api.r_ins_id_;
}

void NetManager::NetFinishInstance(long time, int ins_id){
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

void NetManager::NetSubmitParam(int ss_id, SShipParam ss_param){
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

void NetManager::NetSubmitFBMessage(std::string message){
  boost::thread submit_fb_message(boost::bind(&NetManager::SubmitFBMessage, this, message));
  submit_fb_message.join();
}

void NetManager::SubmitFBMessage(std::string message){
  while(1){
    SubmitFBMessageApi api;

    HttpRequestParameter http_param;
    http_param.b_has_cookies_ = false;
    http_param.str_url_ = base_url_;
    http_param.content_type_ = "application/x-www-form-urlencoded";
    http_param.command_type_ = POST;
    http_param.time_out_ = 300;
    http_param.post_data_ = message;
  
    api.Request(http_param);

    boost::this_thread::sleep(boost::posix_time::millisec(200));
  }
}

void NetManager::NetGetControlData(int ins_id, bool is_get_control){
  boost::thread get_control_data(boost::bind(&NetManager::GetControlData, this, ins_id, is_get_control));
  get_control_data.join();
}

void NetManager::GetControlData(int ins_id, bool is_get_control){
  while(1){
    GetControlApi api(ins_id);

    api.SetControlStatus(is_get_control);
  
    HttpRequestParameter http_param;
    http_param.b_has_cookies_ = false;
    http_param.str_url_ = base_url_;
    http_param.content_type_ = "application/x-www-form-urlencoded";
    http_param.command_type_ = GET;
    http_param.time_out_ = 300;
 
    api.Request(http_param);

    boost::this_thread::sleep(boost::posix_time::millisec(200));
  }
}

