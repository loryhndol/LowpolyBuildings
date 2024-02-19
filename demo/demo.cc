#include <engine/LowpolyGen.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cout << "Usage: ./demo.exe [meshPath]" << std::endl;
    return 0;
  }

  // setup hyper parameters
  LowpolyGen::Config conf;
  conf.epsilonTau = 3e-5;
  conf.N = 50;
  conf.T = 600;
  conf.k = 20;
  conf.epsilon = 1e-9;
  conf.beta = 3.0 / 180.0 * M_PI;

  // generate low-poly mesh models
  LowpolyGen::Generator generator(conf);
  auto result = generator.run(std::string(argv[1]));

  // output low-poly mesh models
  for (int i = 0; i < result.size(); i++) {
    std::cout << "Mesh #" << i << std::endl;
    std::cout << "- vertices: " << result[i].number_of_vertices() << std::endl;
    std::cout << "- faces: " << result[i].number_of_faces() << std::endl;
    std::stringstream ss;
    ss << i;
    std::string idx;
    ss >> idx;
    CGAL::IO::write_OBJ("result/mesh_" + idx + ".obj", result[i]);
  }
  return 0;
}
