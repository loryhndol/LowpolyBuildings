#pragma once

#ifndef _COMMON_H_
#include "Common.h"
#endif  // _COMMON_H_
#include <random>

#include "renderer.h"

namespace LowpolyGen {
double calculateTau(const SurfaceMesh& Mi, const SurfaceMesh& Mo);
}  // namespace LowpolyGen
