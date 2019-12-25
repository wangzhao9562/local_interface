#include <local_interface/FinishInstanceApi.h>

bool FinishInstanceApi::ins_status_ = true;

/* FinishInstanceApi function and interface */
bool FinishInstanceApi::Request(HttpRequestParameter& http_param) {
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
	  is_request = Post(curl, http_param, OnFinishRecv);
        }
	else {
	  return false;
	}

	// cleanup and release
	curl_easy_cleanup(curl);
	curl_global_cleanup();

	return true;
}

bool FinishInstanceApi::DownloadFile(HttpRequestParameter& http_param) {
	/* do nothing */
	return true;
}

size_t FinishInstanceApi::OnFinishRecv(void* buffer, size_t size, size_t nmemb, void* userp) {
	std::string* str = dynamic_cast<std::string*>((std::string*)userp);

        if (str == nullptr || buffer == nullptr) {
            return -1;
	}

	char* p_data = (char*)buffer;
	str->append(p_data, size* nmemb);

	std::cout << *str << std::endl;

	FinishInstanceJson f_ins_json;

	if(f_ins_json.Deserialize(str->c_str())){
	  ins_status_ = f_ins_json.status_;
	}

	return nmemb;
}

/* FinishInstanceJson function and interface */
bool FinishInstanceJson::Default(){
	return false;
}

bool FinishInstanceJson::Null() {
	return false;
}

bool FinishInstanceJson::Bool(bool b) {
	switch (state_) {
	case FinishInstanceJson::kExpectValue:
		status_ = b;
		return true;
	default:
		return false;
	}
}

bool FinishInstanceJson::Int(int i) {
	return false;
}

bool FinishInstanceJson::Uint(unsigned i) {
	return false;
}
bool FinishInstanceJson::Int64(int64_t i) {
	return false;
}

bool FinishInstanceJson::Uint64(uint64_t i) {
	return false;
}

bool FinishInstanceJson::Double(double d) {
	return false;
}

bool FinishInstanceJson::RawNumber(const Ch* str, rapidjson::SizeType len, bool copy) {
	return false;
}

bool FinishInstanceJson::String(const Ch*, rapidjson::SizeType, bool) {
	return false;
}

bool FinishInstanceJson::StartObject() {
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

bool FinishInstanceJson::Key(const Ch* str, rapidjson::SizeType len, bool copy) {
	return true;
}

bool FinishInstanceJson::EndObject(rapidjson::SizeType) {
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

bool FinishInstanceJson::StartArray() {
	return false;
}

bool FinishInstanceJson::EndArray(rapidjson::SizeType) {
	return false;
}

bool FinishInstanceJson::Deserialize(const char* json) {
	rapidjson::Reader reader;
	rapidjson::StringStream ss(json);

	if (reader.Parse(ss, *this)){
		return true;
	}
	return false;
}

std::string FinishInstanceJson::Serialize() {
	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

	writer.StartObject();
	writer.Key("status");
	writer.Bool(status_);
	writer.Key("resp");
	writer.StartObject();
	writer.EndObject();
	writer.EndObject();

	return buffer.GetString();
}
