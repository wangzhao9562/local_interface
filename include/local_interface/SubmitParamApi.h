#pragma once
#include <local_interface/HttpHelper.h>
#include <local_interface/InstanceData.h>
#include <local_interface/JsonTool.h>
#include <local_interface/rapidjson/reader.h>
#include <local_interface/SShipParam.h>
#include <local_interface/HttpApiException.h>

/* TODO: Add ship or vehicle id on SShipParam
 */
struct CSubmitData {
	int id_; // id of ship
	SShipParam param_;
};

/* TODO: Http api which offers the interface to request to submit vehicle parameter to server
 */
class SubmitParamApi : public HttpHelper{
public:
	/* Constructor
	 */
	SubmitParamApi() {};

	/* Out interface to request to submit the vehicle parameter to cloud server
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
	static size_t OnSBParamRecv(void* buffer, size_t size, size_t nmemb, void* userp);

	/* Offer the method to get the post data from input CSubmitData type object
	 * @param ship_param CSubmitData to submit
	 * @return post data in string
	 */
	static std::string GetPostData(CSubmitData submit_data, int ins_id);

private:
	const std::string url_ = "update_param"; // api url
	// CSubmitData sb_data_; // data to submit 
};

