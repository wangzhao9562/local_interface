/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  SubmitFBMessageApi.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/9/1
  * - Brief:     Definition of SubmitFBMessageApi
  *****************************************************************************
**/

#include <local_interface/SubmitFBMessageApi.h>

bool SubmitFBMessageApi::Request(HttpRequestParameter& http_param) {
	bool is_request;
	CURL* curl;
	CURLcode ret;

	ret = curl_global_init(CURL_GLOBAL_ALL);
	curl = curl_easy_init();

	if (curl == nullptr) {
		return false;
	}

	// Post request or Get request
	if (http_param.command_type_ == POST) {
  	  http_param.str_url_ = http_param.str_url_ + url_;
	  is_request = Post(curl, http_param, OnSubmitFBRecv);
        }
	else {
		return false;
	}

	// release
	curl_easy_cleanup(curl);
	curl_global_cleanup();

	return is_request;
}

bool SubmitFBMessageApi::DownloadFile(HttpRequestParameter& http_param){
	/* do nothing */
	return true;
}

size_t SubmitFBMessageApi::OnSubmitFBRecv(void* buffer, size_t size, size_t nmemb, void* userp) {
	std::string* str = dynamic_cast<std::string*>((std::string*)userp);
	
        if (str == nullptr || buffer == nullptr) {
            return -1;
	}

	char* p_data = (char*)buffer;
	str->append(p_data, size* nmemb);

	std::cout << *str << std::endl;

	return nmemb;
}
