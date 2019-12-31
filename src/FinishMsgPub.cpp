/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  FinishMsgPub.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/9/1
  * - Brief:     Node to publish message to finish ordered instance.
  *****************************************************************************
**/

#include <local_interface/FinishMsg.h>
#include <ros/ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "FinishInstance");
  ros::NodeHandle nh;
  
  ros::Publisher finish_msg_pub = nh.advertise<local_interface::FinishMsg>("finishInstance", 1);

  ros::Rate loop_rate(5);

  while(ros::ok()){
    local_interface::FinishMsg finish_msg;
    finish_msg.is_finish = true;
    finish_msg_pub.publish(finish_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
