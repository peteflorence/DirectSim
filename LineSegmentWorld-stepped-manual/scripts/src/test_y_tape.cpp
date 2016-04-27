#include "YTapeGenerator.h"

int main(int argc, char* argv[]) {
  std::cout << "This has been run!" << std::endl;

  YTapeGenerator y_tape_gen;

  Eigen::VectorXd initial_distances = Eigen::VectorXd(21);
  initial_distances.setOnes();

  y_tape_gen.GenerateYTape(initial_distances);

}

