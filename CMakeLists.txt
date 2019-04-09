cmake_minimum_required(VERSION 2.8)
project(vive_provider)

include(FindProtobuf)
find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rhoban_utils
  )

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

# Protobuf generate files with unused parameters
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

# Handling generated headers is based on Dirk Thomas answer on topic:
# - https://answers.ros.org/answers/123416/revisions/
catkin_destinations()
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set (PROTOBUF_MESSAGES
  proto/vive.proto
  )

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${PROTOBUF_MESSAGES})
set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS} PROPERTIES GENERATED TRUE)
set(PROTO_DUMMY_FILE proto_generation)
add_custom_command(
  OUTPUT ${PROTO_DUMMY_FILE}
  COMMAND cp ${PROTO_HEADERS} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  COMMAND touch ${PROTO_DUMMY_FILE}
  DEPENDS ${PROTO_HEADERS}
  )

set(DELEGATE_INCLUDE_DIRS
  include
  ${PROTOBUF_INCLUDE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

catkin_package(
  INCLUDE_DIRS ${DELEGATE_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${PROTOBUF_LIBRARY}
  CATKIN_DEPENDS rhoban_utils
  )

include_directories(${DELEGATE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

set(SOURCES_DIRECTORIES
  src/vive_provider
  )

#Include Sources sub sources
foreach (DIRECTORY ${SOURCES_DIRECTORIES})
    include (${DIRECTORY}/sources.cmake)
    set (PREFIXED_SOURCES)
    foreach (SOURCE ${SOURCES})
        set (PREFIXED_SOURCES ${PREFIXED_SOURCES} ${DIRECTORY}/${SOURCE})
    endforeach (SOURCE)
    set (ALL_SOURCES ${ALL_SOURCES} ${PREFIXED_SOURCES})
endforeach (DIRECTORY)

set(LINKED_LIBRARIES ${PROTOBUF_LIBRARIES} ${catkin_LIBRARIES})

add_library (${PROJECT_NAME} SHARED ${ALL_SOURCES} ${PROTO_SOURCES} ${PROTO_DUMMY_FILE})
target_link_libraries(${PROJECT_NAME} ${LINKED_LIBRARIES})

add_executable(log_manager tools/log_manager.cpp)
target_link_libraries(log_manager ${PROJECT_NAME} ${LINKED_LIBRARIES})
