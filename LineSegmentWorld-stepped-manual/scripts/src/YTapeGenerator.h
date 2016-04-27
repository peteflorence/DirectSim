#include <Eigen/Dense>
#include <iostream>
#include <math.h>


class YTapeGenerator {
public:
  YTapeGenerator() {
    std::cout << "I'm getting constructed" << std::endl;
    SetRaysMatrix();
    initial_state << 0.0, 0.0, 0.0;
  };

  Eigen::MatrixXd GenerateYTape(Eigen::VectorXd initial_distances);
  Eigen::MatrixXd InvertRaycastsToLocations(Eigen::VectorXd initial_distances);
    


private:
  
  void SetRaysMatrix();

  size_t num_steps;
  Eigen::Vector3d initial_state;

  size_t num_rays = 21;
  size_t ray_length = 20;

  double fov = 90.0;
  double fov_rad = fov * M_PI/180.0;
  double angle_min = -fov_rad/2.0;
  double angle_max = fov_rad/2.0;

  Eigen::VectorXd angle_grid = Eigen::VectorXd::LinSpaced(num_rays, angle_min, angle_max);

  Eigen::MatrixXd rays = Eigen::MatrixXd::Zero(3,num_rays);

  double u_max = 2.0;
  double v = 6.0;

  

  


};