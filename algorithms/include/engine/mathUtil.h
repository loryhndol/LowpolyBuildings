#pragma once

#ifndef _COMMON_H_
#include "Common.h"
#endif  // _COMMON_H_
#include <random>

namespace LowpolyGen {
float get_random(unsigned int seed = 0);

Eigen::Vector3f getHemiSpherePoint();
}  // namespace LowpolyGen
