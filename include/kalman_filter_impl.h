#pragma once

#include "kalman_filter.h"

#include <google/protobuf/empty.pb.h>
#include <grpc++/channel.h>

#include <iostream>
#include <kalman/services/kalman_filter.grpc.pb.h>
#include <memory>

namespace kalman {
namespace services {

class KalmanFilterImpl final : public KalmanFilterRPC::Service {
private:
  std::vector<std::unique_ptr<Filter::KalmanFilter>> kalman_filter_instances_;
  bool init_done_;

public:
  explicit KalmanFilterImpl() : init_done_(false){};
  ~KalmanFilterImpl(){};

  ::grpc::Status Create(::grpc::ServerContext *context,
                        const CreateRequest *request,
                        google::protobuf::Empty *response) override;

  ::grpc::Status Tick(::grpc::ServerContext *context,
                      const TickRequest *request,
                      TickResponse *response) override;

  ::grpc::Status Get(::grpc::ServerContext *context,
                     const google::protobuf::Empty *request,
                     TickResponse *response) override;
}; // namespace
} // namespace services
} // namespace kalman
