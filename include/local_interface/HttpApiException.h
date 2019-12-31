/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  HttpApiException.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/8/1
  * - Brief:     Definition of HttpApiException, a set of exceptions of local interface http api classes
  *****************************************************************************
**/

#pragma once
#include <exception>
#include <string>

/* @TODO: Http api exception class 
 */
class HttpApiException : public std::exception {
public:
	/* @breif Constructor
	 */
	HttpApiException(std::string str) : api_fb_msg_(str){};

	/* @breif Override the what() function of exception class
	 * @return Return the exception message
	 */
	virtual const char* what() const noexcept override { return exception_msg.c_str(); };
protected:
	/* @breif Set the exception message
	 * @param message Input exception message
	 */
	void SetExceptMessage(const std::string& message) { exception_msg = message; };

private:
	std::string api_fb_msg_; // feedback message of Http Api
	std::string exception_msg; 
};

/* @TODO: CreateInstanceApi exception
 */
class CreateInstanceException : public HttpApiException {
public:
	CreateInstanceException(std::string str) : HttpApiException(str) { SetExceptMessage("Create instance error: " + str); }
};

/* @TODO: FinishInstance exception
 */
class FinishInstanceException : public HttpApiException {
public:
	FinishInstanceException(std::string str) :HttpApiException(str) { SetExceptMessage("Finish instance error: " + str); }
};

/* @TODO: GetControl exception
 */
class GetControlException : public HttpApiException {
public:
	GetControlException(std::string str) : HttpApiException(str) { SetExceptMessage("Get control command error: " + str); }
};

/* @TODO: SubmitFBMessage exception
 */
class SubmitFBMsgException : public HttpApiException {
public:
	SubmitFBMsgException(std::string str) : HttpApiException(str) { SetExceptMessage("Submit feedback message error: " + str); }
};

/* @TODO: SubmitParamApi exception
 */
class SubmitParamException : public HttpApiException{
public:
	SubmitParamException(std::string str) :  HttpApiException(str) { SetExceptMessage("Submit ship parameter error: " + str); }
};


