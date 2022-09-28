#pragma once

#include "matrix.h"
#include "motion_model.h"
#include <math.h>
#include <memory>

namespace Filter {

class KalmanFilter {

private:
  std::unique_ptr<ConstantAcceleration> motion_model_;
  unsigned int system_states_;

public:
  KalmanFilter(std::unique_ptr<ConstantAcceleration> &motion_model,
               Matrix<float> initial_guess_system,
               Matrix<float> initial_guess_uncertainty);
  ~KalmanFilter(){};
  void set_motion_model(std::unique_ptr<ConstantAcceleration> &motion_model);
  void tick(Matrix<float> measurement_value);
  Matrix<float> get_estimate();

  Matrix<float> estimated_system_state_vector = Matrix<float>(2, 1);
  Matrix<float> predicted_system_state_vector = Matrix<float>(2, 1);

  Matrix<float> estimated_uncertainty = Matrix<float>(2, 2);
  Matrix<float> predicted_uncertainty = Matrix<float>(2, 2);

  Matrix<float> measurement_uncertainty_matrix = Matrix<float>(1, 1);
  Matrix<float> observation_matrix = Matrix<float>(1, 2);
  Matrix<float> measurement_vector = Matrix<float>(1, 1);

  Matrix<float> kalman_gain = Matrix<float>(2, 1);

  Matrix<float> identity_matrix = IdentityMatrix<float>(2);

  float process_noise = 0.0;
  float control_input = 0.0;
  float delta_time = 1;                       /// in seconds
  float variance_velocity = std::pow(0.1, 2); // in m²/s^4
  float random_noise_vector = 0.0;
  float variance_measurement_distance = 9; // in m²

protected:
  void extrapolate_state();
  void extrapolate_uncertainty();
  void calculate_measurement(Matrix<float> measurement_value);
  void calculate_kalman_gain();
  void update_estimate();
  void update_estimate_uncertainty();
  void predict();
  void update();
};
} // namespace Filter
