cmake_minimum_required(VERSION 2.8.3)
project(body_pose)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLIDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
)

add_service_files(
  FILES
  BodyPose.srv
)

add_message_files(
  FILES
  BodyInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_generation cv_bridge message_runtime
)





# add_library(xunfei_tts
#   ${PROJECT_SOURCE_DIR}/src/engines/xunfei/sdk/src/speech_recognizer.c
#   ${PROJECT_SOURCE_DIR}/src/engines/xunfei/sdk/src/xunfei_asr.cpp
#   ${PROJECT_SOURCE_DIR}/src/engines/xunfei/sdk/src/xunfei_tts.cpp
# )

# target_link_libraries(xunfei_tts
#   ${catkin_LIBRARIES}
#   ${PROJECT_SOURCE_DIR}/src/engines/xunfei/sdk/libs/x64/libmsc.so
# )

# set_target_properties(xunfei_tts PROPERTIES OUTPUT_NAME xunfei_tts PREFIX "_" SUFFIX ".so")
# set_target_properties(xunfei_tts PROPERTIES COMPILE_FLAGS "-g -Wno-missing-field-initializers")
# set_target_properties(xunfei_tts PROPERTIES
#   ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
#   LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )