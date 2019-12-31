/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  HttpHelper.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/8/1
  * - Brief:     Definition of HttpHelper, offers the interfaces of base http methods.
  *****************************************************************************
**/
#pragma once
#include <local_interface/curl/curl.h>
#include <string>
#include <iostream>
#include <vector>

#define COOKIE_FILE "cookies.txt"

enum HttpRequestType{POST, GET, DOWNLOAD}; // type of http request

/* relative parameter of http request */
struct HttpRequestParameter {
	HttpRequestParameter() {};
	~HttpRequestParameter() {};

	bool b_read_write_;
	bool b_has_cookies_;
	std::string str_url_;  // url 
	std::string post_data_;  // data to post
	std::string req_ret_context_;
	std::string content_type_;  
	int time_out_;  // time limit
	HttpRequestType command_type_;   // request type
};

typedef std::vector<HttpRequestParameter> HttpParameterVec;

/* struct to write cb data into file */
struct HttpFileParameter {
	HttpFileParameter() { file_stream_ = nullptr; };

	std::string file_name_;
	FILE* file_stream_;
};

/* HttpHelper offer the fundamental function of Http */
class HttpHelper {
public:
	HttpHelper();
	HttpHelper(bool is_has_coolkie);	
	virtual ~HttpHelper();

	virtual bool Request(HttpRequestParameter& http_param) = 0;
	virtual bool DownloadFile(HttpRequestParameter& http_param) = 0;

protected:
	bool Download(CURL* curl, HttpRequestParameter& http_param, void* callback); // definition of basic download function
	bool Post(CURL* curl, HttpRequestParameter& http_param, size_t(*p_func)(void*, size_t, size_t, void*)); // definition of basic post function
	bool Get(CURL* curl, HttpRequestParameter& http_param, size_t(*p_func)(void*, size_t, size_t, void*));  // definition of basic get function
private:
	bool has_cookie_;
};

