#include <local_interface/CreateInstanceApi.h>
#include <local_interface/FinishInstanceApi.h>
#include <local_interface/SubmitFBMessageApi.h>
#include <local_interface/GetControlApi.h>
#include <local_interface/SubmitParamApi.h>
#include <local_interface/CSingleton.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

/* TODO: Net manager which calls all http api
 */
class NetManager{
  friend CSingletonMT<NetManager>;
  friend CSingletonST<NetManager>;

public:
  /* @brief Default constructor
   */
  NetManager(){};

  /* @brief constructor with parameter
   * @param url Input url
   */
  NetManager(std::string url) : base_url_(url){}
 

  /* Deconstructor
   */
  ~NetManager(){};

  /* @brief Call CreateNewInstanceApi to request to create an instance in server
   * @param ins Data of created instance
   */
  void NetCreateNewInstance(InstanceData ins);

  /* @brief Call FinishInstanceApi to request to finish the instance in server
   * @param time The time stamp when request to finish instance
   * @param ins_id Id of the instance
   */
  void NetFinishInstance(long time, int ins_id);

  /* @brief Call SubmitParamApi to request submit param to server
   * @param ss_id ID of ship/vehicle
   * @param ss_param SShipParam type object which store the param of ship/vehicle
   */
  void NetSubmitParam(int ss_id, SShipParam ss_param);

  /* @brief Call SubmitFBMessageApi to request submit fb message to server
   */
  void NetSubmitFBMessage(std::string message);

  /* @brief Call in thread to request submit fb message to server
   * @param message THe fb message 
   */
  void SubmitFBMessage(std::string message);

  /* @brief Call GetControlApi to request control command for server
   */
  void NetGetControlData(int ins_id, bool is_get_control);

  /* @brief Call in thread to request control command message to server
   */
  void GetControlData(int ins_id, bool is_get_control);

  /* @brief Return instance ID
   * @return ID of the created instance
   */
  int GetInstanceID(){
    return ins_id_;
  }

  /* @brief Revise base url from outside
   * @param url Input base url
   */
  void NewBaseUrl(std::string url){
    base_url_ = url;
  }

protected:
  int ins_id_; 
  std::string base_url_;
};
