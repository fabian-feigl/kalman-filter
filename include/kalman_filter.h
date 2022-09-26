#include "matrix.h"
#include <math.h>

namespace Filter {

class KalmanFilter {

public:
  KalmanFilter(Matrix<float> initial_guess_system,
               Matrix<float> initial_guess_uncertainty);
  ~KalmanFilter(){};
  void use_constant_velocity();
  void predict();
  void update();

  Matrix<float> estimated_system_state_vector = Matrix<float>(2, 1);
  Matrix<float> predicted_system_state_vector = Matrix<float>(2, 1);
  Matrix<float> control_matrix = Matrix<float>(2, 1);
  Matrix<float> state_transition_matrix = Matrix<float>(2, 2);

  float process_noise = 0.0;
  float control_input = 0.0;
  float delta_time = 1;                           /// in seconds
  float variance_acceleration = std::pow(0.2, 2); // in m²/s^4

  Matrix<float> estimated_uncertainty = Matrix<float>(2, 2);
  Matrix<float> predicted_uncertainty = Matrix<float>(2, 2);
  Matrix<float> process_noise_matrix = Matrix<float>(2, 2);

protected:
  void extrapolate_state();
  void extrapolate_uncertainty();
  void compute_kalman_gain();
  void update_estimate();
  void update_estimate_uncertainty();
};
} // namespace Filter
