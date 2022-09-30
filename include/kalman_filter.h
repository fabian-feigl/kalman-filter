#pragma once

#include "matrix.h"
#include "motion_model.h"
#include <math.h>
#include <memory>

namespace Filter {

class KalmanFilter {

private:
  std::unique_ptr<MotionModel> motion_model_;
  unsigned int system_states_;
  float variance_measurement_distance_ = 9; // in m²

protected:
  Matrix<float> estimated_system_state_vector;
  Matrix<float> predicted_system_state_vector;

  Matrix<float> estimated_uncertainty;
  Matrix<float> predicted_uncertainty;

  Matrix<float> measurement_uncertainty_matrix = Matrix<float>(1, 1);
  Matrix<float> measurement_vector = Matrix<float>(1, 1);
  Matrix<float> observation_matrix;

  Matrix<float> kalman_gain;
  Matrix<float> identity_matrix;

  void init(Matrix<float> initial_guess_system,
            Matrix<float> initial_guess_uncertainty);
  void set_motion_model(std::unique_ptr<MotionModel> &motion_model);
  void extrapolate_state();
  void extrapolate_uncertainty();
  void calculate_measurement(Matrix<float> measurement_value);
  void calculate_kalman_gain();
  void update_estimate();
  void update_estimate_uncertainty();
  void predict();
  void update();

public:
  KalmanFilter(std::unique_ptr<MotionModel> &motion_model,
               Matrix<float> initial_guess_system,
               Matrix<float> initial_guess_uncertainty);
  ~KalmanFilter(){};
  void tick(Matrix<float> measurement_value);
  void set_measurement_variance(float variance_measurement_distance);
  Matrix<float> get_estimate();
  Matrix<float> get_estimate_uncertainty();

  float process_noise = 0.0;
  float control_input = 0.0;
  float random_noise_vector = 0.0;
};
} // namespace Filter
