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
  kalman_filter_x_ = std::make_unique<Filter::KalmanFilter>(
      motion_model, initial_guess_system, initial_guess_uncertainty);
  kalman_filter_y_ = std::make_unique<Filter::KalmanFilter>(
      motion_model, initial_guess_system, initial_guess_uncertainty);

  kalman_filter_x_->set_measurement_variance(request->measurement_variance());
  kalman_filter_y_->set_measurement_variance(request->measurement_variance());
  return grpc::Status::OK;
}

::grpc::Status KalmanFilterImpl::Tick(::grpc::ServerContext *context,
                                      const TickRequest *request,
                                      TickResponse *response) {
  std::cout << "tick" << std::endl;
  Matrix<float> x_value = Matrix<float>(1, 1);
  x_value(0, 0) = request->measurement().position().x();
  Matrix<float> y_value = Matrix<float>(1, 1);
  y_value(0, 0) = request->measurement().position().y();
  kalman_filter_x_->tick(x_value);
  kalman_filter_y_->tick(y_value);

  float x_return = kalman_filter_x_->get_estimate()(0, 0);
  float y_return = kalman_filter_y_->get_estimate()(0, 0);

  kalman::data::XYPosition position = response->estimate().position();
  position.set_x(x_return);
  position.set_y(y_return);
  return grpc::Status::OK;
}

::grpc::Status KalmanFilterImpl::Get(::grpc::ServerContext *context,
                                     const google::protobuf::Empty *request,
                                     TickResponse *response) {
  std::cout << "Get" << std::endl;
  float x_return = kalman_filter_x_->get_estimate()(0, 0);
  float y_return = kalman_filter_y_->get_estimate()(0, 0);

  kalman::data::XYPosition position = response->estimate().position();
  position.set_x(x_return);
  position.set_y(y_return);
  return grpc::Status::OK;
}
