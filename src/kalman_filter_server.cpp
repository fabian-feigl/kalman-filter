#include "kalman_filter_impl.h"

#include <grpc++/channel.h>
#include <grpc++/create_channel.h>
#include <grpc++/security/server_credentials.h>
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>

void RunServer() {
  std::string server_address("0.0.0.0:5000");
  kalman::services::KalmanFilterImpl service;

  grpc::ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
  server->Wait();
}

int main(int argc, char **argv) {
  RunServer();
  return 0;
}
