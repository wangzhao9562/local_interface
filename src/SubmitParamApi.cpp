#include <local_interface/SubmitParamApi.h>
#include <local_interface/CreateInstanceApi.h>

bool SubmitParamApi::Request(HttpRequestParameter& http_param) {
	bool is_request;
	CURL* curl;
	CURLcode ret;

	// initialization
	ret = curl_global_init(CURL_GLOBAL_ALL);
	curl = curl_easy_init();
	if (curl == nullptr) {
		return false;
	}

	// Post request or Get request
	if (http_param.command_type_ == POST) {
           http_param.str_url_ = http_param.str_url_ + url_;
	   is_request = Post(curl, http_param, OnSBParamRecv);
        }
	else {
		return false;
	}

	// cleanup
	curl_easy_cleanup(curl);
	curl_global_cleanup();

	return true;
}

bool SubmitParamApi::DownloadFile(HttpRequestParameter& http_param) {
	/* do nothing */
	return true;
}

size_t SubmitParamApi::OnSBParamRecv(void* buffer, size_t size, size_t nmemb, void* userp) {
	std::string* str = dynamic_cast<std::string*>((std::string*)userp);

	if (str == nullptr || buffer == nullptr) {
	   return -1;
	}

	char* p_data = (char*)buffer;
	str->append(p_data, size* nmemb);

	std::cout << *str << std::endl;

	return nmemb;
}

std::string SubmitParamApi::GetPostData(CSubmitData submit_data, int ins_id) {
	SShipParam ship_param = submit_data.param_;
	std::string str_post = "lat=" + std::to_string(ship_param.lat_) +
		"&lon=" + std::to_string(ship_param.lon_) +
		"&posX=" + std::to_string(ship_param.pos_x_) +
		"&posY=" + std::to_string(ship_param.pos_y_) +
		"&rud=" + std::to_string(ship_param.rud_) +
		"&phi=" + std::to_string(ship_param.phi_) +
		"&gps_phi=" + std::to_string(ship_param.gps_phi_) +
		"&speed=" + std::to_string(ship_param.speed_) +
		"&gear=" + std::to_string(ship_param.gear_) +
		"&time=" + std::to_string(ship_param.time_) +
		"&kp=" + std::to_string(ship_param.kp_) +
		"&ki=" + std::to_string(ship_param.ki_) +
		"&kd=" + std::to_string(ship_param.kd_) +
		"&k1=" + std::to_string(ship_param.k1_) +
		"&k2=" + std::to_string(ship_param.k2_) +
		"&tem=" + std::to_string(ship_param.temp_) +
		"&pH=" + std::to_string(ship_param.ph_) +
		"&diso=" + std::to_string(ship_param.diso_) +
		"&tur=" + std::to_string(ship_param.tur_) +
		"&con=" + std::to_string(ship_param.con_) +
		"&check_value=" + std::to_string(ship_param.check_value_) +
		"&instanceid=" + std::to_string(ins_id) +
		"&shipid=" + std::to_string(submit_data.id_);
        return str_post;
}
