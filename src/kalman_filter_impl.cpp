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
  kalman_filter_instances_.push_back(std::make_unique<Filter::KalmanFilter>(
      motion_model, initial_guess_system, initial_guess_uncertainty));

  std::cout << "Done" << std::endl;

  // kalman_filter_instances_.back()->set_mea

  // kalman_filter_x_->set_measurement_variance(request->measurement_variance());
  // kalman_filter_y_->set_measurement_variance(request->measurement_variance());
  init_done_ = true;
  return grpc::Status::OK;
}

::grpc::Status KalmanFilterImpl::Tick(::grpc::ServerContext *context,
                                      const TickRequest *request,
                                      TickResponse *response) {

  if (init_done_ == false)
    return ::grpc::Status::CANCELLED;
  std::cout << "tick" << std::endl;
  auto &kalman_filter = kalman_filter_instances_.back();

  Matrix<float> value = Matrix<float>(kalman_filter->get_system_states(), 1);
  value(0, 0) = request->measurement().position().x();

  kalman_filter->tick(value);
  float estimate_f = kalman_filter->get_estimate()(0, 0);

  kalman::data::Estimate estimate;
  kalman::data::XYPosition *position = estimate.mutable_position();
  position->set_x(estimate_f);
  *response->mutable_estimate() = estimate;
  return grpc::Status::OK;
}

::grpc::Status KalmanFilterImpl::Get(::grpc::ServerContext *context,
                                     const google::protobuf::Empty *request,
                                     TickResponse *response) {
  std::cout << "Get" << std::endl;
  if (init_done_ == false)
    return ::grpc::Status::CANCELLED;
  float estimate_f = kalman_filter_instances_.back()->get_estimate()(0, 0);

  kalman::data::Estimate estimate;
  kalman::data::XYPosition *position = estimate.mutable_position();
  position->set_x(estimate_f);
  *response->mutable_estimate() = estimate;
  return grpc::Status::OK;
}
