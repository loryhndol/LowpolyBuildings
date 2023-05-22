#include "engine/mathUtil.h"

namespace LowpolyGen {
float get_random(unsigned int seed) {
  static std::default_random_engine e(seed);
  static std::uniform_real_distribution<float> u(-1, 1);
  return u(e);
}

Eigen::Vector3f getHemiSpherePoint() {
  float u = get_random(time(NULL));
  float v = get_random(time(NULL));
  float r2 = u * u + v * v;
  while (r2 >= 1) {
    u = get_random(time(NULL));
    v = get_random(time(NULL));
    r2 = u * u + v * v;
  }
  float z = std::sqrt(1 - r2);
  float phi = 2 * M_PI * u;
  float x = std::cos(phi) * std::sqrt(r2);
  float y = std::sin(phi) * std::sqrt(r2);
  return Eigen::Vector3f(x, y, z);
}
}  // namespace LowpolyGen
