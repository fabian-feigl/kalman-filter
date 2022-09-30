#pragma once

#include "matrix.h"

#include <memory>

namespace Filter {

struct MotionModelParameter {
  float delta_time;
  unsigned int system_states;
  float process_variance_acceleration;
};

class MotionModel {
private:
  std::unique_ptr<MotionModelParameter> parameters_ = nullptr;

public:
  int maximum_system_states = 3;
  Matrix<float> state_transition_matrix;
  /// HINT: The control matrix will be just zeros as long as there is no
  /// external input to the system.
  Matrix<float> control_matrix;
  Matrix<float> process_noise_matrix;
  MotionModel(){};
  virtual ~MotionModel(){};

  MotionModel(MotionModelParameter &parameters) {
    parameters_ = std::make_unique<MotionModelParameter>(parameters);
    state_transition_matrix =
        Matrix<float>(parameters_->system_states, parameters_->system_states);
    control_matrix = Matrix<float>(parameters_->system_states, 1);
    process_noise_matrix =
        Matrix<float>(parameters_->system_states, parameters_->system_states);
    set_process_noise_matrix();
  }
  MotionModelParameter &get_parameters() { return *parameters_; }
  void set_process_noise_matrix();
  virtual void set_state_transition_matrix() = 0;
};

class ConstantVelocity : public MotionModel {
public:
  ConstantVelocity(MotionModelParameter &parameter) : MotionModel(parameter) {
    set_state_transition_matrix();
  };
  void set_state_transition_matrix();
};

class ConstantAcceleration : public MotionModel {
public:
  ConstantAcceleration(MotionModelParameter &parameter)
      : MotionModel(parameter) {
    set_state_transition_matrix();
  };
  void set_state_transition_matrix();
};

} // namespace Filter
