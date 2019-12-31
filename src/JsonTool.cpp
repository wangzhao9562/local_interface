/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  JsonTool.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/9/1
  * - Brief:     Implementation of json encode and decode interfaces.
  *****************************************************************************
**/

#include <local_interface/JsonTool.h>

/* Json Tool Function and Interface */
JsonToolRead::JsonToolRead(const char* json){
	Init(json);
}

void JsonToolRead::Initialize(const char* json) {
	Init(json);
}

void JsonToolRead::Init(const char* json) {
	json_ = json;
	doc_.Parse(json_);
	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	doc_.Accept(writer);

	is_init_ = true;
}

void JsonToolRead::Reset() {
	doc_.Parse("{}");
	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	doc_.Accept(writer);

	is_init_ = false;
}

bool JsonToolRead::GetValue(std::string key, int &value){
	// uninitialized
	if (!is_init_) {
		return is_init_;
	}

	// if docoment has the key string
	if (doc_.HasMember(key.c_str())) {
		rapidjson::Value& v = doc_[key.c_str()];
		value = v.GetInt();
		return true;
	}
	else
		return false;
}

bool JsonToolRead::GetValue(std::string key, double& value) {
	// uninitialized
	if (!is_init_) {
		return is_init_;
	}

	// if docoment has the key string
	if (doc_.HasMember(key.c_str())) {
		rapidjson::Value& v = doc_[key.c_str()];
		value = v.GetDouble();
		return true;
	}
	else
		return false;
}

bool JsonToolRead::GetValue(std::string key, std::string& value) {
	// uninitialized
	if (!is_init_) {
		return is_init_;
	}

	// if docoment has the key string
	if (doc_.HasMember(key.c_str())) {
		rapidjson::Value& v = doc_[key.c_str()];
		value = v.GetString();
		return true;
	}
	else
		return false;
}

bool JsonToolRead::GetValue(std::string key, bool& value) {
	// uninitialized
	if (!is_init_) {
		return is_init_;
	}

	// if docoment has the key string
	if (doc_.HasMember(key.c_str())) {
		rapidjson::Value& v = doc_[key.c_str()];
		value = v.GetBool();
		return true;
	}
	else
		return false;
}

bool JsonToolRead::GetValue(std::string key, float& value) {
	// uninitialized
	if (!is_init_) {
		return is_init_;
	}

	// if docoment has the key string
	if (doc_.HasMember(key.c_str())) {
		rapidjson::Value& v = doc_[key.c_str()];
		value = v.GetFloat();
		return true;
	}
	else
		return false;
}

bool JsonToolRead::GetValue(std::string key, long& value) {
	// uninitialized
	if (!is_init_) {
		return is_init_;
	}

	// if docoment has the key string
	if (doc_.HasMember(key.c_str())) {
		rapidjson::Value& v = doc_[key.c_str()];
		value = v.GetInt64();
		return true;
	}
	else
		return false;
}

/* Json Tool Function and Interface */
JsonToolWrite::~JsonToolWrite() {
	writer_->EndObject();
	Destroy();
}

void JsonToolWrite::Reset() {
	writer_->EndObject();
	Destroy();
	Init();
}

void JsonToolWrite::Init() {
	buffer_ = new rapidjson::StringBuffer();
	writer_ = new rapidjson::Writer<rapidjson::StringBuffer>(*buffer_);
	writer_->StartObject();
}

void JsonToolWrite::Destroy() {
	if (buffer_ != nullptr)
		delete buffer_;
	if (writer_ != nullptr)
		delete writer_;
}

void JsonToolWrite::SetValue(std::string key, int value) {
	writer_->Key(key.c_str());
	writer_->Int(value);
}

void JsonToolWrite::SetValue(std::string key, double value) {
	writer_->Key(key.c_str());
	writer_->Double(value);
}
void JsonToolWrite::SetValue(std::string key, float value) {
	writer_->Key(key.c_str());
	writer_->Double(value);
}

void JsonToolWrite::SetValue(std::string key, bool value) {
	writer_->Key(key.c_str());
	writer_->Bool(value);
}

void JsonToolWrite::SetValue(std::string key, std::string value) {
	writer_->Key(key.c_str());
	writer_->String(value.c_str());
}

void JsonToolWrite::SetValue(std::string key, long value) {
	writer_->Key(key.c_str());
	writer_->Int64(value);
}

const char* JsonToRapidJsonString(std::string json) {

	return "";
}
