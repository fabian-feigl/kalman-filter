#include "kalman_filter.h"

using namespace Filter;

KalmanFilter::KalmanFilter(std::unique_ptr<ConstantAcceleration> &motion_model,
                           Matrix<float> initial_guess_system,
                           Matrix<float> initial_guess_uncertainty)
    : estimated_system_state_vector(initial_guess_system),
      predicted_system_state_vector(initial_guess_system),
      estimated_uncertainty(initial_guess_uncertainty),
      predicted_uncertainty(initial_guess_uncertainty) {

  set_motion_model(motion_model);
  std::cout << "Initalized Kalman Filter" << std::endl;
  predict();
}

void KalmanFilter::set_motion_model(
    std::unique_ptr<ConstantAcceleration> &motion_model) {

  motion_model_ = std::move(motion_model);
  system_states_ = motion_model_->get_parameters().system_states;
  measurement_uncertainty_matrix(0, 0) = variance_measurement_distance;
  observation_matrix(0, 0) = 1;
}

void KalmanFilter::extrapolate_state() {
  predicted_system_state_vector =
      motion_model_->state_transition_matrix * estimated_system_state_vector +
      motion_model_->control_matrix * control_input + process_noise;

  // std::cout << "Got new predicted system state vector \n"
  //           << predicted_system_state_vector.print() << std::endl;
  return;
}

void KalmanFilter::extrapolate_uncertainty() {
  predicted_uncertainty =
      motion_model_->state_transition_matrix * estimated_uncertainty *
          motion_model_->state_transition_matrix.transpose() +
      motion_model_->process_noise_matrix;

  // std::cout << "Got new uncertainty system state vector \n"
  //           << predicted_uncertainty.print() << std::endl;
  return;
}

void KalmanFilter::calculate_measurement(Matrix<float> measurement_value) {

  measurement_vector =
      observation_matrix * measurement_value + random_noise_vector;
  // std::cout << "Calculate measurement \n"
  //           << measurement_vector.print() << std::endl;
  return;
}

void KalmanFilter::calculate_kalman_gain() {

  Matrix<float> observation_matrix_transposed = observation_matrix.transpose();
  kalman_gain = predicted_uncertainty * observation_matrix_transposed *
                (observation_matrix * predicted_uncertainty *
                     observation_matrix_transposed +
                 measurement_uncertainty_matrix)
                    .inverse();
  std::cout << "Calculate kalman gain \n" << kalman_gain.print() << std::endl;
  return;
}

void KalmanFilter::update_estimate() {

  estimated_system_state_vector =
      predicted_system_state_vector +
      kalman_gain * (measurement_vector -
                     observation_matrix * predicted_system_state_vector);
  std::cout << "update estimate \n"
            << estimated_system_state_vector.print() << std::endl;
  return;
}

void KalmanFilter::update_estimate_uncertainty() {
  Matrix<float> first_part = identity_matrix - kalman_gain * observation_matrix;
  Matrix<float> second_part =
      (identity_matrix - kalman_gain * observation_matrix).transpose();
  Matrix<float> third_part =
      kalman_gain * measurement_uncertainty_matrix * kalman_gain.transpose();
  estimated_uncertainty =
      first_part * predicted_uncertainty * second_part + third_part;
  std::cout << "update estimate uncertainty \n"
            << estimated_uncertainty.print() << std::endl;
  return;
}

void KalmanFilter::predict() {
  extrapolate_state();
  extrapolate_uncertainty();
  return;
}

void KalmanFilter::update() {
  calculate_kalman_gain();
  update_estimate();
  update_estimate_uncertainty();
  return;
}

void KalmanFilter::tick(Matrix<float> measurement_value) {
  calculate_measurement(measurement_value);
  update();
  predict();
}

Matrix<float> KalmanFilter::get_estimate() {
  return estimated_system_state_vector;
}
