/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  CreateInstanceApi.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/9/1
  * - Brief:     Implementation of CreateInstanceApi.
  *****************************************************************************
**/

#include <local_interface/CreateInstanceApi.h>

int CreateInstanceApi::r_ins_id_ = 0;

/* CreateInstanceApi Function and Interface */
bool CreateInstanceApi::Request(HttpRequestParameter& http_param) {
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
	   is_request = Post(curl, http_param, OnCreateRecv);
	}
	else {
		return false;
	}

	// release
	curl_easy_cleanup(curl);
	curl_global_cleanup();

	return is_request;
}


bool CreateInstanceApi::DownloadFile(HttpRequestParameter& http_param_) {
	/* Do nothing */
	return true;
}

size_t CreateInstanceApi::OnCreateRecv(void* buffer, size_t size, size_t nmemb, void* userp) {
// size_t OnWriteData(void* buffer, size_t size, size_t nmemb, void* userp) {
	std::string* str = dynamic_cast<std::string*>((std::string*)userp);

        if (str == nullptr || buffer == nullptr) {
           return -1;
	}

	char* p_data = (char*)buffer;
	str->append(p_data, size* nmemb);

	std::cout << *str << std::endl;

	NewInstanceJson param_json;
	param_json.Deserialize(str->c_str());

        // try{
	if (param_json.status_ == true) {
	  r_ins_id_ = param_json.resp_.id_;
	}

	return nmemb;
}


/*  NewInstanceJson Function and Interface */
bool NewInstanceJson::Default(){
	return false;
}

bool NewInstanceJson::Null() {
	return false;
}

bool NewInstanceJson::Bool(bool b) {
	switch (state_) {
	case NewInstanceJson::kExpectValue:
		status_ = b;
		return true;
	default:
		return false;
	}
}
bool NewInstanceJson::Int(int) {
	return false;
}

bool NewInstanceJson::Uint(unsigned i) {
	switch (state_)
	{
	case NewInstanceJson::kExpectSubValue:
		resp_.id_ = i;
		return true;
	default:
		return false;
	}
}

bool NewInstanceJson::Int64(int64_t) {
	return false;
}

bool NewInstanceJson::Uint64(uint64_t) {
	return false;
}

bool NewInstanceJson::Double(double) {
	return false;
}

bool NewInstanceJson::RawNumber(const Ch* str, rapidjson::SizeType len, bool copy) {
	return false;
}

bool NewInstanceJson::String(const Ch*, rapidjson::SizeType, bool) {
	return false;
}

bool NewInstanceJson::StartObject() {
	switch (state_) {
	case NewInstanceJson::kExpectObjectStart:
		state_ = kExpectValue;
		return true;
	case NewInstanceJson::kExpectValue:
		state_ = kExpectSubValue;
		return true;
	default:
		return false;
	}
}

bool NewInstanceJson::Key(const Ch* str, rapidjson::SizeType len, bool copy) {
	return true;
}

bool NewInstanceJson::EndObject(rapidjson::SizeType) {
	switch (state_)
	{
	case NewInstanceJson::kExpectValue:
		state_ = kExpectNameOrObjectEnd;
		return true;
	case NewInstanceJson::kExpectSubValue:
		state_ = kExpectValue;
		return true;
	default:
		break;
	}
}

bool NewInstanceJson::StartArray() {
	return false;
}

bool NewInstanceJson::EndArray(rapidjson::SizeType) {
	return false;
}

bool NewInstanceJson::Deserialize(const char* json) {
	rapidjson::Reader reader;
	rapidjson::StringStream ss(json);

	if (reader.Parse(ss, *this)) {
		return true;
	}
	return false;
}

std::string NewInstanceJson::Serialize() {
	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

	writer.StartObject();
	writer.Key("status");
	writer.Bool(status_);
	writer.Key("resp");
	writer.StartObject();
	writer.Key("id");
	writer.Uint(resp_.id_);
	writer.EndObject();
	writer.EndObject();

	return buffer.GetString();
}

// /* NewInstanceJson Function and Interface */
// void NewInstanceJson::Deserialize(std::string json) {
// 	JsonToolRead json_reader(json.c_str());
// 
// 	json_reader.GetValue("status", status_);
// 	std::string resp_in_json;
// 	json_reader.GetValue("resp", resp_in_json);
// 	resp_->Deserialize(resp_in_json);
// }
// 
// std::string NewInstanceJson::Serialize() {
// 	JsonToolWrite json_writer;
// 
// 	json_writer.SetValue("status", status_);
// 	std::string resp_json = resp_->Serialize();
// 	json_writer.SetValue("resp", resp_json);
// 
// 	return json_writer.GetJson();
// }
// 
// /* InstanceResp Function and Interface */
// void NewInstanceResp::Deserialize(std::string json) {
// 	JsonToolRead json_reader(json.c_str());
// 	json_reader.GetValue("id", id_);
// }
// 
// std::string NewInstanceResp::Serialize() {
// 	JsonToolWrite json_writer;
// 	json_writer.SetValue("id", id_);
// 
// 	return json_writer.GetJson();
// }
