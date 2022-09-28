#include "kalman_filter.h"
#include <iostream>
#include <random>

void add_gaussian_noise(std::vector<float> &input_values, float mean,
                        float standard_deviation) {
  // Define random generator with Gaussian distribution
  std::default_random_engine generator;
  std::normal_distribution<float> dist(mean, standard_deviation);
  for (auto &x : input_values) {
    x += dist(generator);
  }
  return;
}

float calculate_pose(float x_0, float time, const float velocity = 10.0) {
  return x_0 + velocity * time;
}

std::vector<float> create_measurement_vector(int number_of_measurements,
                                             float standard_deviation,
                                             float x_0) {
  std::vector<float> measurements;
  measurements.push_back(x_0);
  for (int i = 1; i < number_of_measurements; i++) {
    float value = calculate_pose(x_0, i);
    measurements.push_back(value);
  }
  add_gaussian_noise(measurements, 0.0, standard_deviation);
  return measurements;
}

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

  float start_pose = 300.0;
  float standard_deviation_measurement = 3.0;
  int number_of_measurements = 10;
  std::vector<float> measurements = create_measurement_vector(
      number_of_measurements, standard_deviation_measurement, start_pose);
  Matrix<float> measurement = Matrix<float>(2, 1);
  Matrix<float> estimate = Matrix<float>(2, 1);
  float distance;

  for (int i = 0; i < measurements.size(); i++) {

    std::cout << "########   " << i << "    ############" << std::endl;
    measurement(0, 0) = measurements[i];
    kalman_filter.tick(measurement);
    estimate = kalman_filter.get_estimate();

    std::cout << "Real value is  " << calculate_pose(start_pose, i)
              << std::endl;
    std::cout << "Measurement is " << measurement(0, 0) << std::endl;
    std::cout << "Estimate is    " << estimate.get(0, 0) << std::endl;
    distance = estimate.get(0, 0) - calculate_pose(start_pose, i);
    std::cout << "Distance measurement and estimate " << distance << std::endl;
    std::cout << std::endl << std::endl;
  }
}
