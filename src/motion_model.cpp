#include "motion_model.h"
#include <math.h>

using namespace Filter;

void MotionModel::set_process_noise_matrix() {
  /// HINT: Currently process noise is going to be applied on changes in
  /// acceleration.
  MotionModelParameter parameters = this->get_parameters();
  if (parameters.system_states > maximum_system_states) {
    throw std::out_of_range("Maximum system states is " +
                            std::to_string(maximum_system_states) + "! Got " +
                            std::to_string(parameters.system_states));
  }
  if (parameters.system_states >= 1) {
    process_noise_matrix(0, 0) = std::pow(parameters.delta_time, 4) / 4;
  }
  if (parameters.system_states >= 2) {
    process_noise_matrix(0, 1) = std::pow(parameters.delta_time, 3) / 2;
    process_noise_matrix(1, 0) = std::pow(parameters.delta_time, 3) / 2;
    process_noise_matrix(1, 1) = std::pow(parameters.delta_time, 2);
  }
  if (parameters.system_states == 3) {
    process_noise_matrix(0, 2) = std::pow(parameters.delta_time, 2) / 2;
    process_noise_matrix(1, 2) = parameters.delta_time;
    process_noise_matrix(2, 0) = std::pow(parameters.delta_time, 2) / 2;
    process_noise_matrix(2, 1) = parameters.delta_time;
    process_noise_matrix(2, 2) = 1;
  }
  process_noise_matrix =
      process_noise_matrix * parameters.process_variance_acceleration;
  return;
}

void ConstantVelocity::set_state_transition_matrix() {
  MotionModelParameter parameters = this->get_parameters();
  if (parameters.system_states > maximum_system_states) {
    throw std::out_of_range("Maximum system states is " +
                            std::to_string(maximum_system_states) + "! Got " +
                            std::to_string(parameters.system_states));
  }
  if (parameters.system_states >= 1) {
    state_transition_matrix(0, 0) = 1.0;
  }
  if (parameters.system_states >= 2) {
    state_transition_matrix(0, 1) = parameters.delta_time;
    state_transition_matrix(1, 0) = 0.0;
    state_transition_matrix(1, 1) = 1.0;
  }
  // HINT: Adding more than 2 system state dimensions does not affect the
  // state transition matrix of the constant velocity model!
}

void ConstantAcceleration::set_state_transition_matrix() {
  MotionModelParameter parameters = this->get_parameters();
  if (parameters.system_states > maximum_system_states) {
    throw std::out_of_range("Maximum system states is " +
                            std::to_string(maximum_system_states) + "! Got " +
                            std::to_string(parameters.system_states));
  }
  if (parameters.system_states >= 1) {
    state_transition_matrix(0, 0) = 1.0;
  }
  if (parameters.system_states >= 2) {
    state_transition_matrix(0, 1) = parameters.delta_time;
    state_transition_matrix(1, 0) = 0.0;
    state_transition_matrix(1, 1) = 1.0;
  }
  if (parameters.system_states == 3) {
    state_transition_matrix(0, 2) = 0.5 * std::pow(parameters.delta_time, 2);
    state_transition_matrix(1, 2) = parameters.delta_time;
    state_transition_matrix(2, 0) = 0.0;
    state_transition_matrix(2, 1) = 0.0;
    state_transition_matrix(2, 2) = 1.0;
  }
}
