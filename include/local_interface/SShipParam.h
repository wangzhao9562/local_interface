#pragma once

/* TODO: Store and offer parameters of one ship and vehicle
 */
struct SShipParam {
	/* Default constructor
	 */
	SShipParam();

	/* Construtor with input
	 */
	SShipParam(double lat, double lon, double alti, double lat_ori, double lon_ori, double pos_x, double pos_y, double phi, double gps_phi, 
		        float rud_ang, float speed, int gear, long time,
		        float kp, float ki, float kd, float k1, float k2,
		        double temp, double ph, double diso, double tur, int con, double check_value);

        /* Update check value when the data members are revised from outside
         */
        void UpdateCheckValue(){
           check_value_ = lat_ + lon_ + pos_x_ + pos_y_ + phi_ + gps_phi_ + rud_ + speed_  + kp_ + ki_ + kd_ + k1_ + k2_ + temp_ + ph_ + diso_ + tur_ + con_;
        }

	/* Override operator=
	 * @param ship_param Input SShipParam type object to copy with
	 */
	void operator=(SShipParam ship_param);

	double lat_;
	double lon_;
        double alti_;
  
        double lat_ori_;
        double lon_ori_;
  
	double pos_x_;
	double pos_y_;

	double f_error_;

	double phi_;
	double gps_phi_;

	float rud_;
	float speed_;
	int gear_;
	long time_;

	float kp_;
	float ki_;
	float kd_;
	float k1_;
	float k2_;

	double temp_;
	double ph_;
	double diso_;
	double tur_;
	int con_;
	double check_value_;
};
