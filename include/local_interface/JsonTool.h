#pragma once
#include <local_interface/rapidjson/rapidjson.h>
#include <local_interface/rapidjson/document.h>
#include <local_interface/rapidjson/stringbuffer.h>
#include <local_interface/rapidjson/writer.h>
#include <string>

/* 
 * TODO: Read input json string, offer the interface to get value of input key
 */
class JsonToolRead {
public:
	/* Default Constructor 
	 */
	JsonToolRead() { is_init_ = false; }

	/* Constructor
	 * @param json The input json string
	 */
	JsonToolRead(const char* json);

	/* Deconstructor
	 */
	~JsonToolRead() {};

	/* Interface to reset Reader
	 */
	void Reset();

	/* Interface to initialize reader
	 */
	void Initialize(const char* json);

	/* Get int type value according to input key
	 * @param key Input key to get int type value from json string
	 * @param value The value get from json string
	 * @return Whether get the value in right way
	 */
	bool GetValue(std::string key, int& value);

	/* Get string type value according to input key
     * @param key Input key to get string type value from json string
     * @param value The value get from json string
     * @return Whether get the value in right way
     */
	bool GetValue(std::string key, double& value);

	/* Get string type value according to input key
     * @param key Input key to get int type value from json string
     * @param value The value get from json string
     * @return Whether get the value in right way
     */
	bool GetValue(std::string key, std::string& value);

	/* Get bool type value according to input key
     * @param key Input key to get int type value from json string
     * @param value The value get from json string
     * @return Whether get the value in right way
     */
	bool GetValue(std::string key, bool& value);

	/* Get float type value according to input key
     * @param key Input key to get int type value from json string
     * @param value The value get from json string
     * @return Whether get the value in right way
     */
	bool GetValue(std::string key, float& value);

	/* Get long type value according to input key
     * @param key Input key to get int type value from json string
     * @param value The value get from json string
     * @return Whether get the value in right way
     */
	bool GetValue(std::string key, long& value);

private:
	/* Init function
     */
	void Init(const char* json);

private:
	rapidjson::Document doc_; // DOM object to store json string
	const char* json_;
	bool is_init_;
};

/* TODO: Offer the interface to build a json string
 */
class JsonToolWrite {
public:
	/* Constructor
     */
	JsonToolWrite() { Init(); };

	/* Deconstructor
	 */
	~JsonToolWrite();

	/* Interface to reset Writer
	 */
	void Reset();

	/* Add input key and int type value into built json string
	 * @param key Input key
	 * @param value Input int type value of the key
	 */
	void SetValue(std::string key, int value);

	/* Add input key and int type value into built json string
     * @param key Input key
     * @param value Input int type value of the key
     */
	void SetValue(std::string key, double value);

	/* Add input key and float type value into built json string
     * @param key Input key
     * @param value Input float type value of the key
     */
	void SetValue(std::string key, float value);

	/* Add input key and bool type value into built json string
     * @param key Input key
     * @param value Input bool type value of the key
     */
	void SetValue(std::string key, bool value);

	/* Add input key and string type value into built json string
     * @param key Input key
     * @param value Input string type value of the key
     */
	void SetValue(std::string key, std::string value);

	/* Add input key and long type value into built json string
     * @param key Input key
     * @param value Input long type value of the key
     */
	void SetValue(std::string key, long value);

	/* Out interface to get built json
	 */
	std::string GetJson() {
		if (buffer_ != nullptr) {
			std::string built_json = buffer_->GetString();
			built_json = built_json + "}";
			return built_json;
		}
		return "";
	}

private:
	/* Initialize function, called in the Constructor
	 */
	void Init();

	/* Release class member, called in Reset and Deconstructor
     */
	void Destroy();

private:
	rapidjson::StringBuffer* buffer_; // Json string buffer
	rapidjson::Writer<rapidjson::StringBuffer>* writer_;
};

/* Transform common json string to rapidjson string
 * @param json Input common json str
 * @return Json str in rapidjson format
 */
const char* JsonToRapidJsonString(std::string json);

/* TODO: The fundamental class which offers the interfaces to realize deserialize and serialize between class and Json string
 */
class JsonObject {
public:
	/* Constructor
	 */
	JsonObject() {}

	/* Deconstructor
	 */
	virtual ~JsonObject() {}

	/* Interface to realize deserialize from input json string
	 */
	virtual void Deserialize(std::string json) = 0;

    /* Interface to realize serialize from class to Json string
	 * return The Json string get from class 
	 */
	virtual std::string Serialize() = 0;

// private:
// 	JsonTool tool_;
};

struct Test {
	int a_;
	int b_;
};
