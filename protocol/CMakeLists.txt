find_package(Protobuf 3.0.0 REQUIRED)
find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
find_program(_PROTOBUF_PROTOC protoc)


# Protobuf Message files
set(PROTOBUF_DEFS_DIR ${PROJECT_SOURCE_DIR}/protocol)
message(STATUS "foo include dir: ${PROTOBUF_DEFS_DIR}")

# List of public protobuf sources
set(PROTOBUF_SOURCES
    kalman/data/data.proto
    kalman/data/motion_model.proto
    kalman/services/kalman_filter.proto)
set(PROTOBUF_GENERATED_CPP)
set(PROTOBUF_GENERATED_HEADER)

set(PROTOBUF_GRPC_SOURCES
    kalman/services/kalman_filter.proto)
set(PROTOBUF_GENERATED_GRPC_CPP)
set(PROTOBUF_GENERATED_GRPC_HEADER)

# Generate C++ .pb.cc and pb. files
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

# Generate C++ .grpc.pb.cc and .grpc.pb.h
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

add_library(kalman_filter_proto
  ${PROTOBUF_GENERATED_CPP}
  ${PROTOBUF_GENERATED_HEADER}
  ${PROTOBUF_GENERATED_GRPC_CPP}
  ${PROTOBUF_GENERATED_GRPC_HEADER})

target_include_directories(kalman_filter_proto PRIVATE ${PROJECT_BINARY_DIR}/include)

target_link_libraries(kalman_filter_proto
  grpc++_reflection
  grpc++
  protobuf::libprotobuf
  protoc)

# Generate Python 
find_package(Python COMPONENTS Interpreter REQUIRED)
set(OUTPUT "${GENERATED_PY_DIR}/dist/kalman_filter_proto.tar.gz")

set(PROTOBUF_GENERATED_PY)
set(GRPC_GENERATED_PY)

set(GENERATED_PY_DIR ${PROJECT_BINARY_DIR}/python)
foreach(FILE ${PROTOBUF_SOURCES})
    string(REPLACE ".proto" "_pb2.py" FILE_PY ${FILE})
    add_custom_command(
        OUTPUT ${GENERATED_PY_DIR}/${FILE_PY}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${GENERATED_PY_DIR}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE} ${PROTOBUF_DEFS_DIR}/${FILE} --python_out=${GENERATED_PY_DIR} --proto_path=${PROTOBUF_DEFS_DIR}
        DEPENDS ${FILE}
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}    
        )
    list(APPEND PROTOBUF_GENERATED_PY ${GENERATED_PY_DIR}/${FILE_PY})

endforeach()

#Generate Python gRPC
foreach(FILE ${PROTOBUF_GRPC_SOURCES})
    string(REPLACE ".proto" "_pb2_grpc.py" FILE_GRPC ${FILE})
    add_custom_command(
        OUTPUT ${GENERATED_PY_DIR}/${FILE_GRPC}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${GENERATED_PY_DIR}
        COMMAND python -m grpc_tools.protoc ${PROTOBUF_DEFS_DIR}/${FILE} --grpc_python_out=${GENERATED_PY_DIR} --python_out=${GENERATED_PY_DIR}  --proto_path=${PROTOBUF_DEFS_DIR} 
        DEPENDS ${FILE}
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    )
    list(APPEND GRPC_GENERATED_PY ${GENERATED_PY_DIR}/${FILE_GRPC})
endforeach()

configure_file(${PROJECT_SOURCE_DIR}/python/setup.py ${GENERATED_PY_DIR}/setup.py)
add_custom_command(
    OUTPUT ${OUTPUT}
    # Use Python script to generate __init__.py files recursively:
    COMMAND python ${PROJECT_SOURCE_DIR}/python/create_package.py --package ${GENERATED_PY_DIR}
    COMMAND python ${PROJECT_SOURCE_DIR}/python/setup.py build
    COMMAND ${CMAKE_COMMAND} -E remove_directory dist
    COMMAND python ${PROJECT_SOURCE_DIR}/python/setup.py sdist
    DEPENDS ${PROTOBUF_GENERATED_PY} ${GRPC_GENERATED_PY}
    WORKING_DIRECTORY ${GENERATED_PY_DIR}
)

add_custom_target(python_protocol ALL DEPENDS ${OUTPUT})
