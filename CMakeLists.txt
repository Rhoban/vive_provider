cmake_minimum_required(VERSION 3.10.2)
project(vive_provider)

find_package(Protobuf REQUIRED)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

# Protobuf generate files with unused parameters
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS
  proto/vive.proto
)
add_library(${PROJECT_NAME} SHARED
  ${PROTO_SOURCES}
  ${PROTO_HEADERS}
)
add_subdirectory(src)
target_link_libraries(${PROJECT_NAME} PUBLIC  rhoban_utils protobuf::libprotobuf)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/..
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(log_manager tools/log_manager.cpp)
target_link_libraries(log_manager ${PROJECT_NAME})
