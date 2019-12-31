/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  InstanceData.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/9/1
  * - Brief:     Implementation of InstanceData.
  *****************************************************************************
**/

#include <local_interface/InstanceData.h>

InstanceData::InstanceData(std::string name, std::string desp, int amount, std::string shape, long time) : name_(name), 
                                                                                                           desp_(desp), 
	                                                                                                       amount_(amount), 
	                                                                                                       shape_(shape),
	                                                                                                       time_(time) 
{
	bool is_init_ = true;
}

InstanceData::~InstanceData() {};

void InstanceData::Deserialize(std::string json) {
	JsonToolRead json_tool(json.c_str());
	json_tool.GetValue("name", name_);
	json_tool.GetValue("desp", desp_);
	json_tool.GetValue("amount", amount_);
	json_tool.GetValue("shape", shape_);
	json_tool.GetValue("time", time_);
}

std::string InstanceData::Serialize() {
	if (is_init_) {
		JsonToolWrite json_tool;

		json_tool.SetValue("name", name_);
		json_tool.SetValue("desp", desp_);
		json_tool.SetValue("amount", amount_);
		json_tool.SetValue("shape", shape_);
		json_tool.SetValue("time", time_);

		return json_tool.GetJson();
	}
	else {
		return "";
	}
}

/* deserialize funtion for class InstanceData */
// InstanceData InstanceDeserilize(std::string str) {
// 
// }
// 
// /* serialize function for class InstanceData */
// std::string InstanceSerialize(InstanceData instance) {
// 
// }
