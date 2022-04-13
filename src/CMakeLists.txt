cmake_minimum_required(VERSION 3.16.3)

target_sources(${PROJECT_NAME} PRIVATE
  vive_provider/udp_message_manager.cpp
  vive_provider/utils.cpp
)
