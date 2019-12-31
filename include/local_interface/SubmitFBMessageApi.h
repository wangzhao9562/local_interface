/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  SubmitFBMessageApi.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/8/1
  * - Brief:     Definition of SubmitFBMessageApi, used to receive control status message from cloud server
  *****************************************************************************
**/

#pragma once
#include <local_interface/HttpHelper.h>
#include <local_interface/SShipParam.h>
#include <local_interface/HttpApiException.h>

/* TODO: Http api which offers the interface to request to submit feedback message to server
 */
class SubmitFBMessageApi : public HttpHelper {
public:
	/* Constructor
	 */
	SubmitFBMessageApi() {};
	//SubmitFBMessageApi(std::string status) { status_ = status; };

	/* Out interface to request to submit the feedback message to cloud server
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
	static size_t OnSubmitFBRecv(void* buffer, size_t size, size_t nmemb, void* userp);

private:
	const std::string url_ = "update_fbmessage"; // api url
	// std::string status_; // status message to submit
};

