# kalman-filter
Kalman Filter Implementation in C++ 

Available motion models for Constant Velocity (CV) and Constant Acceleration (CA).

# Build

This project can be built like an usual CMake project

```bash
mkdir build && cd build
cmake ..
make
```

# Execute

## CPP example application

The `kalman_filter_example.cpp` is an exemplary application to demonstrate the functionality of kalman filter library. 

CMake generates an executable in `build/src` called `kalman_filter_example`.

## Kalman Filter Server

The repository offers a gRPC based kalman filter  microservice application.

The gRPC server is generated as an executable in `build/src` called `kalman_filter_server`.

In the `example` folder a Jupyter Notebook demonstrates the usage with `Python`.
In order to get all necessary libraries for Protobuf and gRPC you have to install the pip package that is generated with CMake.

```bash
cd build/python/dist
pip install kalman_filter_proto-*.tar.gz
```






