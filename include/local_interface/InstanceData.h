#pragma once
#include <string>
#include <local_interface/JsonTool.h>

/* TODO: Store and offer the details of a Instance 
 */
class InstanceData : JsonObject {
public:
	/* Default Contructor
	 */
	InstanceData() { is_init_ = false; };

	/* Initializer
	 * @param name 
	 */
	InstanceData(std::string name, std::string desp, int amount, std::string shape, long time);

	/* Deconstructor
	 */
	~InstanceData();

	/* Interface to realize deserialize from Json string to class
	 * @param json Json string in stl string format
	 */
	void Deserialize(std::string json) override;

	/* Interface to realize serialize from class to Json string
	 * @return Json string in stl string format 
	 */
	std::string Serialize() override;
	
	/* Override operator = for class InstanceData
	 * @param instance Input InstanceData object
	 */
	void operator=(InstanceData instance) {
		if (this != &instance) {
			this->name_ = instance.name_;
			this->desp_ = instance.desp_;
			this->amount_ = instance.amount_;
			this->shape_ = instance.shape_;
			this->time_ = instance.time_;

			this->is_init_ = instance.is_init_;
		}
	}

public:
	std::string name_; // name of instance
	std::string desp_; // description
	int amount_;       // amount of object in instance
	std::string shape_;  // type of object
	long time_;          // time stamp

private:
	bool is_init_;
};

// /* deserialize funtion for class InstanceData */
// InstanceData InstanceDeserilize(std::string str);
// 
// /* serialize function for class InstanceData */
// std::string InstanceSerialize(InstanceData instance);
