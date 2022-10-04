find_package(Protobuf 3.0.0 REQUIRED)
find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)

# Protobuf Message files
set(PROTOBUF_DEFS_DIR ${PROJECT_SOURCE_DIR}/protocol)
message(STATUS "foo include dir: ${PROTOBUF_DEFS_DIR}")

# List of public protobuf sources
set(PROTOBUF_SOURCES
    kalman_filter/data/data.proto
    kalman_filter/data/motion_model.proto
    kalman_filter/services/kalman_filter.proto)
set(PROTOBUF_GENERATED_CPP)
set(PROTOBUF_GENERATED_HEADER)

set(PROTOBUF_GRPC_SOURCES
    kalman_filter/services/kalman_filter.proto)
set(PROTOBUF_GENERATED_GRPC_CPP)
set(PROTOBUF_GENERATED_GRPC_HEADER)

set(GENERATED_CPP_DIR ${PROJECT_BINARY_DIR}/include)
foreach(FILE ${PROTOBUF_SOURCES})
    string(REPLACE ".proto" ".pb.cc" FILE_CC ${FILE})
    string(REPLACE ".proto" ".pb.h" FILE_H ${FILE})
    add_custom_command(
        OUTPUT ${GENERATED_CPP_DIR}/${FILE_CC}
        OUTPUT ${GENERATED_CPP_DIR}/${FILE_H}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${GENERATED_CPP_DIR}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE} ${PROTOBUF_DEFS_DIR}/${FILE} --cpp_out=${GENERATED_CPP_DIR} --proto_path=${PROTOBUF_DEFS_DIR}
        DEPENDS ${FILE}
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    )
    list(APPEND PROTOBUF_GENERATED_CPP ${GENERATED_CPP_DIR}/${FILE_CC})
    list(APPEND PROTOBUF_GENERATED_HEADER ${GENERATED_CPP_DIR}/${FILE_H})
endforeach()

foreach(FILE ${PROTOBUF_GRPC_SOURCES})
    string(REPLACE ".proto" ".grpc.pb.cc" GRPC_FILE_CC ${FILE})
    string(REPLACE ".proto" ".grpc.pb.h" GRPC_FILE_H ${FILE})
    add_custom_command(
        OUTPUT ${GENERATED_CPP_DIR}/${GRPC_FILE_CC}
        OUTPUT ${GENERATED_CPP_DIR}/${GRPC_FILE_H}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${GENERATED_CPP_DIR}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE} ${PROTOBUF_DEFS_DIR}/${FILE} --proto_path=${PROTOBUF_DEFS_DIR}
        --grpc_out "${GENERATED_CPP_DIR}" --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        DEPENDS ${FILE}
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    )
    list(APPEND PROTOBUF_GENERATED_GRPC_CPP ${GENERATED_CPP_DIR}/${GRPC_FILE_CC})
    list(APPEND PROTOBUF_GENERATED_GRPC_HEADER ${GENERATED_CPP_DIR}/${GRPC_FILE_H})
endforeach()

include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# rg_grpc_proto
add_library(kalman_filter_proto
  ${PROTOBUF_GENERATED_CPP}
  ${PROTOBUF_GENERATED_HEADER}
  ${PROTOBUF_GENERATED_GRPC_CPP}
  ${PROTOBUF_GENERATED_GRPC_HEADER})

target_include_directories(kalman_filter_proto PRIVATE ${PROJECT_BINARY_DIR}/include)

target_link_libraries(kalman_filter_proto
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF})
