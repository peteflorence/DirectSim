#include "YTapeGenerator.h"

void YTapeGenerator::SetRaysMatrix() {

  for (size_t i = 0; i < num_rays; i++) {
    std::cout << i << std::endl;

    rays(0,i) = cos(angle_grid(i));
    rays(1,i) = -sin(angle_grid(i));
  }

  std::cout << rays << std::endl;

};

Eigen::MatrixXd YTapeGenerator::GenerateYTape(Eigen::VectorXd initial_distances) {

  std::cout << "I'm getting called too" << std::endl;

  Eigen::MatrixXd y_tape = Eigen::MatrixXd(num_steps + 1, num_rays);
  
  Eigen::MatrixXd initial_locations = InvertRaycastsToLocations(initial_distances);

  return y_tape;

};

Eigen::MatrixXd YTapeGenerator::InvertRaycastsToLocations(Eigen::VectorXd initial_distances) {

  Eigen::MatrixXd locations = Eigen::MatrixXd::Zero(num_rays,3);
  Eigen::Vector3d origin;
  origin << initial_state(0), initial_state(1), 0.0;

  std::cout << " " << std::endl;

  Eigen::Vector3d ray;
  for (size_t i = 0; i < num_rays; i++) {
   ray = rays.col(i);
   //std::cout << "ray is" << ray << std::endl;
   //Eigen::Vector3d rotated_ray = RotateByTheta(ray, 0.0)

  }

  std::cout << "Inverting complete" << std::endl;
  return locations;

};

// Eigen::Vector3d RotateByTheta(Eigen::Vector3d, double theta) {

// };

