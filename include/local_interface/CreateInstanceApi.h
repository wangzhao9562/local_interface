#pragma once
#include <local_interface/HttpHelper.h>
#include <local_interface/InstanceData.h>
#include <local_interface/JsonTool.h>
#include <local_interface/rapidjson/reader.h>
#include <local_interface/HttpApiException.h>

/* TODO: Http api which offers the interface to request to create an instance in cloud server
 */
class CreateInstanceApi : public HttpHelper {
public:
	// /* Default Constructor
	//  */
	// CreateInstanceApi() { is_init_ = false;}

	/* Constructor with input
	 */
	CreateInstanceApi(InstanceData instance) { instance_ = instance; is_init_ = true; }

	/* Out interface to request to create an instance in cloud server
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
	static size_t OnCreateRecv(void* buffer, size_t size, size_t nmemb, void* userp);

public:
	static int r_ins_id_;
	
private:
	const std::string url_ = "start_instance";
	InstanceData instance_;
	bool is_init_;
};

/* TODO: Main description of a new instance
 */
struct NewInstanceResp : public rapidjson::BaseReaderHandler<rapidjson::UTF8<>, NewInstanceResp>{
	/* Default constructor
	 */
	NewInstanceResp() { id_ = 0; };

	/* Funtions used to deserialization
	 */
	bool Null() { return false; };
	bool Bool(bool b) { return false; }
	bool Int(int i) { id_ = i; return true; };
	bool Uint(unsigned i) { id_ = i; return true; };
	bool Int64(int64_t i) { id_ = i; return true; };
	bool Uint64(uint64_t i) { id_ = i; return true; };
	bool Double(double d) { return false; };
	bool RawNumber(const Ch* str, rapidjson::SizeType length, bool copy) { return true;; };
	bool String(const Ch* str, rapidjson::SizeType length, bool copy) { return false; };
	bool StartObject() { return true; };
	bool Key(const Ch* str, rapidjson::SizeType length, bool copy) { return true; };
	bool EndObject(rapidjson::SizeType memberCount) { return true; };
	bool StartArray() { return false; };
	bool EndArray(rapidjson::SizeType elementCount) { return false; };

	/* Override operator=
     * @param input NewInstanceResp type object
     */
	void operator=(NewInstanceResp ins_resp) {
		id_ = ins_resp.id_;
	}

	unsigned int id_; // id of instance
};

/* TODO: Json analysis class for CreateInstanceApi
 */
struct NewInstanceJson : public rapidjson::BaseReaderHandler<rapidjson::UTF8<>, NewInstanceJson> {

	/* Default constructor
	 */
	NewInstanceJson() : state_(kExpectObjectStart) {}

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
	void operator=(NewInstanceJson ins_json) {
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

// /* TODO: Main description of a new instance
//  */
// class NewInstanceResp : JsonObject {
// 	friend class NewInstanceJson;
// 	friend class CreateInstanceApi;
// 	// friend size_t OnWriteData(void* buffer, size_t size, size_t nmemb, void* userp);
// public:
// 	/* Default constructor
// 	 */
// 	NewInstanceResp() { id_ = 0; }
// 
// 	/* Copy constructor
// 	 * @param resp NewInstanceResp type object to copy with
// 	 */
// 	NewInstanceResp(NewInstanceResp& resp) { id_ = resp.id_; }
// 
// 	/* Out interface to realize serialization from class to Json string
// 	 * @param json input json string
// 	 */
// 	void Deserialize(std::string json) override;
// 
// 	/* Out interface to realize serialization from class to Json string
// 	 * @return output json string of class
// 	 */
// 	std::string Serialize() override;
// 
// 	/* Override operator =
// 	 * @param resp input NewInstanceResp type object to copy with
// 	 */
// 	void operator=(NewInstanceResp resp) {
// 		id_ = resp.id_;
// 	}
// 
// private:
// 	int id_; // id of instance
// };


// /* TODO: Json analysis class for CreateInstanceApi 
//  */
// class NewInstanceJson : JsonObject {
// 	friend class CreateInstanceApi;
// 	// friend size_t OnWriteData(void* buffer, size_t size, size_t nmemb, void* userp);
// public:
// 	/* Default constructor
// 	 */
// 	NewInstanceJson() { status_ = false; resp_ = new NewInstanceResp(); }
// 
// 	/* Copy constructor
// 	 * @param ins_json NewInstanceJson type object to copy with
// 	 */
// 	NewInstanceJson(NewInstanceJson& ins_json) { status_ = ins_json.status_; resp_ = ins_json.resp_; }
// 
// 	/* Deconstructor
// 	 */
// 	~NewInstanceJson() { if (resp_ != nullptr) delete resp_; }
// 
// 	/* Out interface to realize deserialization from Json string to class
// 	 * @param json input json string
// 	 */
// 	void Deserialize(std::string json) override;
// 
// 	/* Out interface to realize serialization from class to Json string
// 	 * @return output json string of class
// 	 */
// 	std::string Serialize() override;
// 
// 	/* Override operator =
// 	 * @param json input NewInstanceJson type object to copy with
// 	 */
// 	void operator=(NewInstanceJson json) {
// 		status_ = json.status_;
// 		resp_ = resp_;
// 	}
// 
// private:
// 	bool status_; // status of instance running in cloud server
// 	NewInstanceResp* resp_;
// };

// /* Callback function of Request, receive its response and deserialize the received data into class
//  * @param buffer buffer to store the received data
//  * @param size size of the buffer
//  * @param nmemb size of the received data
//  * @param userp input ptr decided by user
//  */
// static size_t OnWriteData(void* buffer, size_t size, size_t nmemb, void* userp);

