/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  GetControlApi.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/9/1
  * - Brief:     Implementation of GetControlApi.
  *****************************************************************************
**/

#include <local_interface/GetControlApi.h>

std::string GetControlApi::get_command_ = "";

/* GetControlApi Function and Interface */
bool GetControlApi::Request(HttpRequestParameter& http_param) {
	bool is_request;
	CURL* curl;
	CURLcode ret;

	/* initialize */
	ret = curl_global_init(CURL_GLOBAL_ALL);
	curl = curl_easy_init();
	if (curl == nullptr) {
		return false;
	}

	// Post request or Get request
	if (http_param.command_type_ == GET && is_get_control_) {
           http_param.str_url_ = http_param.str_url_ + url_ + std::to_string(ins_id_);
	   is_request = Get(curl, http_param, OnGetControlRecv);
        }
	else {
           return false;
	}

	// release
	curl_easy_cleanup(curl);
	curl_global_cleanup();

	return is_request;
}

bool GetControlApi::DownloadFile(HttpRequestParameter& http_param) {
	/* do nothing */
	return true;
}

void GetControlApi::SetControlStatus(bool is_get_control) {
	is_get_control_ = is_get_control;
}

size_t GetControlApi::OnGetControlRecv(void* buffer, size_t size, size_t nmemb, void* userp) {
	std::string* str = dynamic_cast<std::string*>((std::string*)userp);

	if (str == nullptr || buffer == nullptr) {
            return false;
        }

	char* p_data = (char*)buffer;
	str->append(p_data, size* nmemb);

	// std::cout << *str << std::endl;

	ControlJson json;
	json.Deserialize(str->c_str());

	if (json.state_ == true) {
     	  get_command_ = json.resp_;
	}

	return nmemb;
}

/* ControlJson Function and Interface */
bool ControlJson::Default() {
	return false;
}

bool ControlJson::Null() {
	return false;
}

bool ControlJson::Bool(bool b) {
	switch (state_)
	{
	case ControlJson::kExpectValue:
		status_ = b;
		return true;
	default:
		return false;
	}
}

bool ControlJson::Int(int) {
	return false;
}

bool ControlJson::Uint(unsigned) {
	return false;
}

bool ControlJson::Int64(int64_t) {
	return false;
}

bool ControlJson::Uint64(uint64_t) {
	return false;
}

bool ControlJson::Double(double) {
	return false;
}

bool ControlJson::RawNumber(const Ch* str, rapidjson::SizeType len, bool copy) {
	resp_ = str;
	return true;
}

bool ControlJson::String(const Ch* str, rapidjson::SizeType len, bool copy) {
	resp_ = str;
	return true;
}

bool ControlJson::StartObject() {
	switch (state_)
	{
	case ControlJson::kExpectObjectStart:
		state_ = kExpectValue;
	    return true;
	default:
		return false;
	}
}

bool ControlJson::Key(const Ch* str, rapidjson::SizeType len, bool copy) {
	return true;
}

bool ControlJson::EndObject(rapidjson::SizeType){
	switch (state_)
	{
	case ControlJson::kExpectValue:
		state_ = kExpectNameOrObjectEnd;
		return true;
	default:
		return false;
	}
}

bool ControlJson::StartArray() {
	return false;
}

bool ControlJson::EndArray(rapidjson::SizeType) {
	return false;
}

bool ControlJson::Deserialize(const char* json) {
	rapidjson::Reader reader;
	rapidjson::StringStream ss(json);

	if (reader.Parse(ss, *this)) {
		return true;
	}
	return false;
}

std::string ControlJson::Serialize() {
	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

	writer.StartObject();
	writer.Key("status");
	writer.Bool(status_); 
	writer.Key("resp");
	writer.String(resp_.c_str());
	writer.EndObject();

	return buffer.GetString();
}
