#pragma once

#include <vector>

#include "objLoader.h"

class Scene {
 private:
  std::vector<MeshModel> models;

 public:
  void loadMesh(const char* modelPath, bool centerAligned,
                bool noSlash = false);
  void loadMesh(std::vector<MeshModel>& models);
  MeshModel& get(int id);
  int numOfModels();
  void clear();
};
