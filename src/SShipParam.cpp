/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  SShipParam.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/9/1
  * - Brief:     Implementation of SShipParam
  *****************************************************************************
**/

#include <local_interface/SShipParam.h>

SShipParam::SShipParam() {
	lat_ = 0;
	lon_ = 0;
        alti_ = 0;

        lat_ori_ = 0;
        lon_ori_ = 0;  

	pos_x_ = 0;
	pos_y_ = 0;

	f_error_ = 0;

	phi_ = 0;
	gps_phi_ = 0;

	rud_ = 0;
	speed_ = 0;
	gear_ = 0;
	time_ = 0;

	kp_ = 0;
	ki_ = 0;
	kd_ = 0;
	k1_ = 0;
	k2_ = 0;

	temp_ = 0;
	ph_ = 0;
	diso_ = 0;
	tur_ = 0;
	con_ = 0;

        check_value_ = lat_ + lon_ + pos_x_ + pos_y_ + phi_ + gps_phi_ + rud_ + speed_ + gear_ +
                       time_ + kp_ + ki_ + kd_ + k1_ + k2_ + temp_ + ph_ + diso_ + tur_ + con_;
}

SShipParam::SShipParam(double lat, double lon, double alti, double lat_ori, double lon_ori, double pos_x, double pos_y, double phi, double gps_phi,
	float rud_ang, float speed, int gear, long time,
	float kp, float ki, float kd, float k1, float k2,
	double temp, double ph, double diso, double tur, int con, double check_value) : lat_(lat), lon_(lon), alti_(alti), lat_ori_(lat_ori), lon_ori_(lon_ori), pos_x_(pos_x), pos_y_(pos_y), phi_(phi), gps_phi_(gps_phi),
	                                                            rud_(rud_ang), speed_(speed), gear_(gear), time_(time),
	                                                            kp_(kp), ki_(ki), kd_(kd), k1_(k1), k2_(k2),
	                                                            temp_(temp), ph_(ph), diso_(diso), tur_(tur), con_(con), check_value_(check_value) {}

void SShipParam::operator=(SShipParam ship_param) {
	lat_ = ship_param.lat_;
	lon_ = ship_param.lon_;
        alti_ = ship_param.alti_;        
 
        lat_ori_ = ship_param.lat_ori_;
        lon_ori_ = ship_param.lon_ori_;

	pos_x_ = ship_param.pos_x_;
	pos_y_ = ship_param.pos_y_;

	f_error_ = ship_param.f_error_;

	phi_ = ship_param.phi_;
	gps_phi_ = ship_param.gps_phi_;

	rud_ = ship_param.rud_;
	speed_ = ship_param.speed_;
	gear_ = ship_param.gear_;
	time_ = ship_param.time_;

	kp_ = ship_param.kp_;
	ki_ = ship_param.ki_;
	kd_ = ship_param.kd_;
	k1_ = ship_param.k1_;
	k2_ = ship_param.k2_;

	temp_ = ship_param.temp_;
	ph_ = ship_param.ph_;
	diso_ = ship_param.diso_;
	tur_ = ship_param.tur_;
	con_ = ship_param.con_;
 
        check_value_ = ship_param.check_value_;
}
