#pragma once

#include "matrix.h"

#include <memory>

namespace Filter {

struct MotionModelParameter {
  float delta_time;
  unsigned int system_states;
  float process_variance;
};

class MotionModel {
private:
  std::unique_ptr<MotionModelParameter> parameters_ = nullptr;

public:
  Matrix<float> state_transition_matrix;
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
  }
  MotionModelParameter &get_parameters() { return *parameters_; }
  virtual void apply_model() = 0;
};

class ConstantVelocity : public MotionModel {
public:
  ConstantVelocity(MotionModelParameter &parameter) : MotionModel(parameter) {
    apply_model();
  };
  void apply_model();
};

class ConstantAcceleration : public MotionModel {
public:
  ConstantAcceleration(MotionModelParameter &parameter)
      : MotionModel(parameter) {
    apply_model();
  };
  void apply_model();
};

} // namespace Filter
