#include "kalman_filter.h"

using namespace Filter;

KalmanFilter::KalmanFilter(Matrix<float> initial_guess_system,
                           Matrix<float> initial_guess_uncertainty)
    : estimated_system_state_vector(initial_guess_system),
      predicted_system_state_vector(initial_guess_system),
      estimated_uncertainty(initial_guess_uncertainty),
      predicted_uncertainty(initial_guess_uncertainty) {

  std::cout << "Initalized Kalman Filter" << std::endl;
}

void KalmanFilter::use_constant_velocity() {
  state_transition_matrix(0, 0) = 1.0;
  state_transition_matrix(0, 1) = delta_time;
  state_transition_matrix(1, 0) = 0.0;
  state_transition_matrix(1, 1) = 1.0;

  control_matrix(0, 0) = 0.0;
  control_matrix(1, 0) = 0.0;

  process_noise_matrix(0, 0) = std::pow(delta_time, 4) / 4;
  process_noise_matrix(0, 1) = std::pow(delta_time, 3) / 2;
  process_noise_matrix(1, 0) = std::pow(delta_time, 3) / 2;
  process_noise_matrix(1, 1) = std::pow(delta_time, 2);
  process_noise_matrix = process_noise_matrix * variance_acceleration;
}

void KalmanFilter::extrapolate_state() {
  predicted_system_state_vector =
      state_transition_matrix * estimated_system_state_vector +
      control_matrix * control_input + process_noise;

  std::cout << "Got new predicted system state vector "
            << predicted_system_state_vector.print() << std::endl;
  return;
}

void KalmanFilter::extrapolate_uncertainty() {
  predicted_uncertainty = state_transition_matrix * estimated_uncertainty *
                              state_transition_matrix.transpose() +
                          process_noise_matrix;

  std::cout << "Got new uncertainty system state vector "
            << predicted_uncertainty.print() << std::endl;
  return;
}

void KalmanFilter::predict() {
  extrapolate_state();
  extrapolate_uncertainty();
  return;
}
