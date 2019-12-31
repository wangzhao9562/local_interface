/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  GetControlApi.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/8/1
  * - Brief:     Definition of GetControlApi, api ot get control command from cloud server.
  *****************************************************************************
**/

#pragma once
#include <local_interface/HttpHelper.h>
#include <local_interface/InstanceData.h>
#include <local_interface/rapidjson/reader.h>
#include <local_interface/HttpApiException.h>

/* TODO:  Http api which offers the interface to get control command from cloud server
 */
class GetControlApi : public HttpHelper {
public:
	/* Constructor
	 */
	GetControlApi(int ins_id) { ins_id_ = ins_id; is_get_control_ = true; }

	/* Out interface to request to get the control command in cloud server
     * @param http_param Input HttpRequestParameter type object
     * @return Is request successful
     */
	bool Request(HttpRequestParameter& http_param) override;

	/* Out interface to download file from cloud server, however, in this Api this function does nothing in this api
	 * @param http_param Input HttpRequestParameter type object
	 * @return Is request successful
	 */
	bool DownloadFile(HttpRequestParameter& http_param) override;

	/* Out interface to set is_get_control_ property
	 * @param is_get_control input which decides whether the api executes the GET request to server
	 */
	void SetControlStatus(bool is_get_control);

	/* Callback function of Request, receive its response and deserialize the received data into class
	 * @param buffer buffer to store the received data
	 * @param size size of the buffer
	 * @param nmemb size of the received data
	 * @param userp input ptr decided by user
	 */
	static size_t OnGetControlRecv(void* buffer, size_t size, size_t nmemb, void* userp);

public:
	static std::string get_command_;

private:
	const std::string url_ = "get_control/";
	int ins_id_;
	bool is_get_control_;

};


/* TODO: Response from cloud when submiting the parameter
 */
struct ControlJson : public rapidjson::BaseReaderHandler<rapidjson::UTF8<>, ControlJson> {
	/* Default constructor
	 */
	ControlJson() : state_(kExpectObjectStart), status_(false), resp_(""){}

	/* Funtions used to deserialization
     */
	bool Default();
	bool Null();
	bool Bool(bool);
	bool Int(int);
	bool Uint(unsigned);
	bool Int64(int64_t);
	bool Uint64(uint64_t);
	bool Double(double);
	bool RawNumber(const Ch* str, rapidjson::SizeType len, bool copy);
	bool String(const Ch*, rapidjson::SizeType, bool);
	bool StartObject();
	bool Key(const Ch* str, rapidjson::SizeType len, bool copy);
	bool EndObject(rapidjson::SizeType);
	bool StartArray();
	bool EndArray(rapidjson::SizeType);

	/* Deserialize input json to class
	 * @param json input json string
	 * @return if deserialization success
	 */
	bool Deserialize(const char* json);

	/* Serialize class to json string
	 * @return return the result
	 */
	std::string Serialize();

	enum State {
		kExpectObjectStart,
		kExpectNameOrObjectEnd,
		kExpectValue,
	}state_; // enum class of state

	bool status_;
	std::string resp_;
};  
