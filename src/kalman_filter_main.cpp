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

  Matrix<float> measurement = Matrix<float>(2, 1);
  measurement(0, 0) = -393.66;
  kalman_filter.tick(measurement);
  measurement(0, 0) = -375.93;
  kalman_filter.tick(measurement);
}
