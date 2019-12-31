/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  FinishInstanceApi.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/8/1
  * - Brief:     Definition of base FinishInstanceApi, used to finish an created instance in cloud server.
  *****************************************************************************
**/

#pragma once
#include <local_interface/HttpHelper.h>
#include <local_interface/InstanceData.h>
#include <local_interface/CreateInstanceApi.h>
#include <local_interface/JsonTool.h>
#include <local_interface/rapidjson/reader.h>
#include <local_interface/HttpApiException.h>

/* TODO:  Http api which offers the interface to request to finish the instance in cloud server
 */
class FinishInstanceApi : public HttpHelper {
public:
	/* Constructor
	 */
	FinishInstanceApi(int ins_id, long time) { ins_id_ = ins_id; time_ = time; };
	
	/* Out interface to request to finish the instance in cloud server
     * @param http_param Input HttpRequestParameter type object
     * @return Is request successful
     */
	bool Request(HttpRequestParameter& http_param) override;

	/* Out interface to download file from cloud server, however, in this Api this function does nothing in this api
	 * @param http_param Input HttpRequestParameter type object
	 * @return Is request successful
	 */
	bool DownloadFile(HttpRequestParameter& http_param) override;

	/* Callback function of Request, receive its response and deserialize the received data into class
     * @param buffer buffer to store the received data
     * @param size size of the buffer
     * @param nmemb size of the received data
     * @param userp input ptr decided by user
     */
	static size_t OnFinishRecv(void* buffer, size_t size, size_t nmemb, void* userp);

public:
	static bool ins_status_; // status of finished instance recv from server

private:
	const std::string url_ = "finish_instance"; // url of this api
	int ins_id_; // id of the instance to be finished
	long time_; // time stamp when request to finish an instance
};

/* TODO: Description of status of finished instance
 */
struct FinishInstanceJson : public rapidjson::BaseReaderHandler<rapidjson::UTF8<>, FinishInstanceJson>{
	/* Default constructor
     */
	FinishInstanceJson() : state_(kExpectObjectStart) {}

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

	/* Override operator=
	 * @param input NewInstanceJson type object
	 */
	void operator=(FinishInstanceJson ins_json) {
		status_ = ins_json.status_;
		state_ = ins_json.state_;
		resp_ = ins_json.resp_;
	}

	enum State {
		kExpectObjectStart,
		kExpectNameOrObjectEnd,
		kExpectValue,
		kExpectSubObjectStart,
		kExpectSubObjectEnd,
		kExpectSubValue,
	}state_; // enum class of state

	bool status_; // instance status
	NewInstanceResp resp_;
};
