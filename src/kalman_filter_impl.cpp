#include "kalman_filter_impl.h"
#include <google/protobuf/empty.pb.h>
#include <google/protobuf/util/time_util.h>

using namespace kalman::services;

::grpc::Status KalmanFilterImpl::Create(::grpc::ServerContext *context,
                                        const CreateRequest *request,
                                        google::protobuf::Empty *response) {
  std::cout << "Create" << std::endl;
  Filter::MotionModelParameter *parameters = new Filter::MotionModelParameter();
  parameters->delta_time =
      request->motion_model().motion_model_parameters().delta_time();
  parameters->process_variance_acceleration =
      request->motion_model()
          .motion_model_parameters()
          .process_variance_acceleration();
  parameters->system_states =
      request->motion_model().motion_model_parameters().system_states();

  Matrix<float> initial_guess_system =
      Matrix<float>(parameters->system_states, 1);
  Matrix<float> initial_guess_uncertainty =
      Matrix<float>(parameters->system_states, parameters->system_states);

  for (int i = 0; i < initial_guess_uncertainty.get_x_dimension(); i++) {
    initial_guess_uncertainty(i, i) = 500;
  }
  std::unique_ptr<Filter::MotionModel> motion_model;

  if (request->motion_model().motion_model_type() ==
      ::kalman::motion_model::MotionModel_Type::
          MotionModel_Type_CONSTANT_ACCELERATION) {
    motion_model = std::make_unique<Filter::ConstantAcceleration>(*parameters);
  } else if (request->motion_model().motion_model_type() ==
             ::kalman::motion_model::MotionModel_Type::
                 MotionModel_Type_CONSTANT_VELOCITY) {
    motion_model = std::make_unique<Filter::ConstantVelocity>(*parameters);
  }
  kalman_filter_instances_[request->identifier()] =
      (std::make_unique<Filter::KalmanFilter>(
          motion_model, initial_guess_system, initial_guess_uncertainty));

  kalman_filter_instances_[request->identifier()]->set_measurement_variance(
      request->measurement_variance());
  std::cout << request->identifier() << " successfully initialized"
            << std::endl;

  return grpc::Status::OK;
}

::grpc::Status KalmanFilterImpl::Tick(::grpc::ServerContext *context,
                                      const TickRequest *request,
                                      TickResponse *response) {
  std::unordered_map<std::string,
                     std::unique_ptr<Filter::KalmanFilter>>::const_iterator
      iterator = kalman_filter_instances_.find(request->identifier());

  if (iterator == kalman_filter_instances_.end())
    return ::grpc::Status::CANCELLED;
  else {
    auto &kalman_filter = iterator->second;

    Matrix<float> value = Matrix<float>(kalman_filter->get_system_states(), 1);
    value(0, 0) = request->measurement().value();

    kalman_filter->tick(value);
    float estimate_position = kalman_filter->get_estimate()(0, 0);
    float estimate_velocity = kalman_filter->get_estimate()(1, 0);
    float estimate_acceleration = kalman_filter->get_estimate()(2, 0);

    kalman::data::Estimate estimate;
    estimate.set_position(estimate_position);
    estimate.set_velocity(estimate_velocity);
    estimate.set_acceleration(estimate_acceleration);
    *response->mutable_estimate() = estimate;
    // std::cout << "ticked " << iterator->first << std::endl;
    return grpc::Status::OK;
  }
}

::grpc::Status KalmanFilterImpl::Get(::grpc::ServerContext *context,
                                     const GetRequest *request,
                                     TickResponse *response) {

  std::unordered_map<std::string,
                     std::unique_ptr<Filter::KalmanFilter>>::const_iterator
      iterator = kalman_filter_instances_.find(request->identifier());

  if (iterator == kalman_filter_instances_.end())
    return ::grpc::Status::CANCELLED;
  else {
    auto &kalman_filter = iterator->second;
    float estimate_position = kalman_filter->get_estimate()(0, 0);
    float estimate_velocity = kalman_filter->get_estimate()(1, 0);
    float estimate_acceleration = kalman_filter->get_estimate()(2, 0);

    kalman::data::Estimate estimate;
    estimate.set_position(estimate_position);
    estimate.set_velocity(estimate_velocity);
    estimate.set_acceleration(estimate_acceleration);
    *response->mutable_estimate() = estimate;
    // std::cout << "get " << iterator->first << std::endl;
    return grpc::Status::OK;
  }
}
