#pragma once

#ifndef _COMMON_H_
#include "Common.h"
#endif  // _COMMON_H_

namespace LowpolyGen {
std::vector<SurfaceMesh> calculateParetoSet(const SurfaceMesh& mesh);
}  // namespace LowpolyGen
