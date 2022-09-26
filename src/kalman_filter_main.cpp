#include "kalman_filter.h"
#include <iostream>

int main(int argc, char *argv[]) {
  std::cout << "Hello from Kalman Filter" << std::endl;

  Matrix<float> initial_guess_system = Matrix<float>(2, 1);
  initial_guess_system(0, 0) = 0; // x pose
  initial_guess_system(1, 0) = 0; // x velocity

  Matrix<float> initial_guess_uncertainty = Matrix<float>(2, 2);
  initial_guess_uncertainty(0, 0) = 500;
  initial_guess_uncertainty(1, 1) = 500;

  Filter::KalmanFilter kalman_filter(initial_guess_system,
                                     initial_guess_uncertainty);
  kalman_filter.use_constant_velocity();
  kalman_filter.predict();
  kalman_filter.predict();

  std::cout << kalman_filter.state_transition_matrix.print();
  std::cout << kalman_filter.state_transition_matrix.transpose().print();
}
