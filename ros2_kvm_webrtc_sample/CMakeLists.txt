cmake_minimum_required(VERSION 3.5)
project(ros2_kvs_webrtc_sample)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(COMPILER_OPTIONS -Wall -Wextra -Wpedantic)
endif()

add_subdirectory(third_party/amazon-kinesis-video-streams-webrtc-sdk-c)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(
  NvEncoderCuda SHARED
  third_party/video-sdk-samples/Samples/NvCodec/NvEncoder/NvEncoderCuda.cpp
  third_party/video-sdk-samples/Samples/NvCodec/NvEncoder/NvEncoder.cpp)
target_include_directories(NvEncoderCuda
                           PUBLIC third_party/video-sdk-samples/Samples
                                  third_party/video-sdk-samples/Samples/NvCodec
                                  /usr/local/cuda/include)
target_link_libraries(NvEncoderCuda PUBLIC cuda)
set_target_properties(
  NvEncoderCuda
  PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
             $<TARGET_PROPERTY:NvEncoderCuda,INTERFACE_INCLUDE_DIRECTORIES>)

add_library(
  amazon-kinesis-video-streams-webrtc-sdk-c-samples SHARED
  third_party/amazon-kinesis-video-streams-webrtc-sdk-c/samples/Common.c)
target_compile_definitions(amazon-kinesis-video-streams-webrtc-sdk-c-samples
                           PRIVATE -DCMAKE_DETECTED_CACERT_PATH)
target_compile_options(amazon-kinesis-video-streams-webrtc-sdk-c-samples
                       PRIVATE -Wno-unused-function)
target_include_directories(
  amazon-kinesis-video-streams-webrtc-sdk-c-samples
  PUBLIC
    third_party/amazon-kinesis-video-streams-webrtc-sdk-c/src/include
    third_party/amazon-kinesis-video-streams-webrtc-sdk-c/open-source/include
    third_party/amazon-kinesis-video-streams-webrtc-sdk-c/samples)
target_link_libraries(amazon-kinesis-video-streams-webrtc-sdk-c-samples
                      PUBLIC kvsWebrtcClient kvsWebrtcSignalingClient
                             kvspicUtils)
target_link_directories(
  amazon-kinesis-video-streams-webrtc-sdk-c-samples PUBLIC
  third_party/amazon-kinesis-video-streams-webrtc-sdk-c/open-source/lib)
set_target_properties(
  amazon-kinesis-video-streams-webrtc-sdk-c-samples
  PROPERTIES
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
    $<TARGET_PROPERTY:amazon-kinesis-video-streams-webrtc-sdk-c-samples,INTERFACE_INCLUDE_DIRECTORIES>
  )

add_executable(${PROJECT_NAME}_node src/main.cpp)
add_definitions(
  -DKVS_CA_CERT_PATH="${CMAKE_SOURCE_DIR}/third_party/amazon-kinesis-video-streams-webrtc-sdk-c/certs/cert.pem"
  )
ament_target_dependencies(${PROJECT_NAME}_node
                          "geometry_msgs"
                          "rclcpp"
                          "sensor_msgs")
target_compile_options(${PROJECT_NAME}_node PRIVATE ${COMPILER_OPTIONS})
target_include_directories(${PROJECT_NAME}_node PRIVATE /usr/local/cuda/include)
target_link_libraries(${PROJECT_NAME}_node
                      dl
                      NvEncoderCuda
                      amazon-kinesis-video-streams-webrtc-sdk-c-samples)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS NvEncoderCuda amazon-kinesis-video-streams-webrtc-sdk-c-samples
        LIBRARY DESTINATION lib)

file(
  GLOB OPEN_SOURCE_LIBRARIES
  "third_party/amazon-kinesis-video-streams-webrtc-sdk-c/open-source/lib/*.so*")
install(FILES ${OPEN_SOURCE_LIBRARIES} DESTINATION lib)
install(
  FILES
    ${CMAKE_SOURCE_DIR}/third_party/amazon-kinesis-video-streams-webrtc-sdk-c/certs/cert.pem
  DESTINATION share)
