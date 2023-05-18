#pragma once
#ifndef _COMMON_H_
#include "Common.h"
#endif  // _COMMON_H_

// reading OBJ
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

#include "CarvedMeshGenerator.h"
#include "VisualHullConstructor.h"
#include "calculateParetoSet.h"
#include "calculateTau.h"

namespace LowpolyGen {
SurfaceMesh readMeshFromOBJ(const std::string& OBJPath);

class Generator {
 private:
  const Config& _conf;

 public:
  Generator(const Config& conf);

  std::vector<SurfaceMesh> run(std::string& meshPath);
};
}  // namespace LowpolyGen
