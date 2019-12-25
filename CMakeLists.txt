cmake_minimum_required(VERSION 2.8.3)
project(local_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  hust_arms_usv_msgs
)

add_message_files(
  FILES
  InstanceId.msg
  FinishMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/NetManager.cfg
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime dynamic_reconfigure hust_arms_usv_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(CreateInstanceApi
  src/HttpHelper.cpp
  src/InstanceData.cpp
  src/JsonTool.cpp
  src/CreateInstanceApi.cpp	
)

add_library(FinishInstanceApi
  src/HttpHelper.cpp
  src/InstanceData.cpp
  src/JsonTool.cpp
  src/CreateInstanceApi.cpp
  src/FinishInstanceApi.cpp
)

add_library(SubmitFBMessageApi
  src/HttpHelper.cpp
  src/SShipParam.cpp
  src/SubmitFBMessageApi.cpp
)

add_library(GetControlApi
  src/HttpHelper.cpp
  src/InstanceData.cpp
  src/JsonTool.cpp
  src/GetControlApi.cpp
)

add_library(SubmitParamApi
  src/HttpHelper.cpp
  src/InstanceData.cpp
  src/JsonTool.cpp
  src/SShipParam.cpp
  src/SubmitParamApi.cpp
)

add_library(NetManage 
  src/HttpHelper.cpp
  src/InstanceData.cpp
  src/JsonTool.cpp
  src/SShipParam.cpp
  src/SubmitParamApi.cpp
  src/FinishInstanceApi.cpp
  src/CreateInstanceApi.cpp
  src/GetControlApi.cpp
  src/SubmitFBMessageApi.cpp
  src/NetManager.cpp
  src/NetManagerROS.cpp
  src/CtrlCommandSolver.cpp
)


link_directories(
  /usr/share/curl-7.65.3/lib/
)

set(LIB_CURL
  curl
)

target_link_libraries(CreateInstanceApi ${catkin_LIBRARIES})
target_link_libraries(FinishInstanceApi ${catkin_LIBRARIES})
target_link_libraries(SubmitFBMessageApi ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(GetControlApi ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(SubmitParamApi ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(NetManage ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(CreateInstanceApi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(FinishInstanceApi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(SubmitFBMessageApi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(GetControlApi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(SubmitParamApi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(NetManage ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(create_instance_node src/CreateInstanceNode.cpp)
add_executable(finish_instance_node src/FinishInstanceNode.cpp)
add_executable(submit_fbmessage_node src/SubmitFBMessageNode.cpp)
add_executable(get_control_node src/GetControlNode.cpp)
add_executable(submit_param_node src/SubmitParamNode.cpp)
add_executable(fmsg_publisher src/FinishMsgPub.cpp)

target_link_libraries(create_instance_node
  ${catkin_LIBRARIES}
)
target_link_libraries(finish_instance_node
  ${catkin_LIBRARIES}
)
target_link_libraries(submit_fbmessage_node
  ${catkin_LIBRARIES}
)
target_link_libraries(get_control_node
  ${catkin_LIBRARIES}
)
target_link_libraries(submit_param_node
  ${catkin_LIBRARIES}
)
target_link_libraries(fmsg_publisher 
  ${catkin_LIBRARIES}
)

target_link_libraries(create_instance_node CreateInstanceApi ${catkin_LIBRARIES})
target_link_libraries(finish_instance_node FinishInstanceApi ${catkin_LIBRARIES})
target_link_libraries(submit_fbmessage_node SubmitFBMessageApi ${catkin_LIBRARIES})
target_link_libraries(get_control_node GetControlApi ${catkin_LIBRARIES})
target_link_libraries(submit_param_node SubmitParamApi ${catkin_LIBRARIES})
target_link_libraries(fmsg_publisher ${catkin_LIBRARIES})

target_link_libraries(create_instance_node ${catkin_LIBRARIES} ${LIB_CURL})
target_link_libraries(finish_instance_node ${catkin_LIBRARIES} ${LIB_CURL})
target_link_libraries(submit_fbmessage_node ${catkin_LIBRARIES} ${LIB_CURL})
target_link_libraries(get_control_node ${catkin_LIBRARIES} ${LIB_CURL})
target_link_libraries(submit_param_node ${caktin_LIBRARIES} ${LIB_CURL})
