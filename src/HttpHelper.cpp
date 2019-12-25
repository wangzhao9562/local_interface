#define _CRT_SECURE_NO_WARNINGS
#include <local_interface/HttpHelper.h>
#include <exception>

/* constructor */
HttpHelper::HttpHelper() : has_cookie_(false) {}
HttpHelper::HttpHelper(bool is_has_cookie_) : has_cookie_(is_has_cookie_){}

/* deconstructor */
HttpHelper::~HttpHelper() {}

/* realization of http get */
bool HttpHelper::Get(CURL* curl, HttpRequestParameter& http_param, size_t(*p_func)(void*, size_t, size_t, void*)) {
	CURLcode ret;
	if (curl == nullptr) {
		return 0;
	}

	// set properties and options for curl
	curl_easy_setopt(curl, CURLOPT_URL, http_param.str_url_.c_str());
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, http_param.time_out_);

	if (http_param.b_has_cookies_) {
		if (!http_param.b_read_write_)
			curl_easy_setopt(curl, CURLOPT_COOKIEJAR, COOKIE_FILE);
		else
			curl_easy_setopt(curl, CURLOPT_COOKIEFILE, COOKIE_FILE);
	}
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, p_func);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void*)&http_param.req_ret_context_);

	// perform
	ret = curl_easy_perform(curl);

	return ret == CURLE_OK;
}

/* realization of http post */
bool HttpHelper::Post(CURL* curl, HttpRequestParameter& http_param, size_t(*p_func)(void*, size_t, size_t, void*)) {
	CURLcode ret;
	if (curl == nullptr) {
		return 0;
	}

	// set properties and options for curl
	curl_easy_setopt(curl, CURLOPT_URL, http_param.str_url_.c_str());
	curl_easy_setopt(curl, CURLOPT_POST, 1L);
	struct curl_slist* headers = nullptr;
	const std::string content_type = "Content-type:" + http_param.content_type_;
	headers = curl_slist_append(headers, content_type.c_str());
	curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, http_param.time_out_);
	
	if (http_param.b_has_cookies_) {
		if (!http_param.b_read_write_)
			curl_easy_setopt(curl, CURLOPT_COOKIEJAR, COOKIE_FILE);
		else
			curl_easy_setopt(curl, CURLOPT_COOKIEFILE, COOKIE_FILE);
	}

	std::string str_content;

	if (!http_param.post_data_.empty()) {
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, http_param.post_data_.c_str());
	}
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, p_func);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void*)&http_param.req_ret_context_);

	// perform
	ret = curl_easy_perform(curl);
	
	return ret == CURLE_OK;
}

/* realization of file download */
bool HttpHelper::Download(CURL* curl, HttpRequestParameter& http_param, void* callback) {
	if (curl == nullptr) {
		return 0;
	}

	// set properties and options for curl
	curl_easy_setopt(curl, CURLOPT_URL, http_param.str_url_.c_str());
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
	curl_easy_setopt(curl, CURLOPT_COOKIEFILE, COOKIE_FILE);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback);

	curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
	curl_easy_setopt(curl, CURLOPT_POST, false);

	HttpFileParameter http_file;
	http_file.file_name_ = http_param.req_ret_context_;

	curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void*)&http_file);

	if (!http_param.post_data_.empty())
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, http_param.post_data_.c_str());

	// perform
	CURLcode res = curl_easy_perform(curl);

	if (http_file.file_stream_ != nullptr) {
		fclose(http_file.file_stream_);
	}

	return res == CURLE_OK;
}
