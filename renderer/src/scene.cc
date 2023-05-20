#include "scene.h"

void Scene::loadMesh(const char* modelPath, bool centerAligned, bool noSlash) {
  MeshModel mesh(modelPath, noSlash);
  if (centerAligned) {
    mesh.centerAligned();
  }
  models.push_back(mesh);
}

void Scene::loadMesh(std::vector<MeshModel>& newModels) {
  models = newModels;
  for (int i = 0; i < models.size(); i++) {
    models[i].centerAligned();
  }
}

MeshModel& Scene::get(int id) {
  if (models.size() == 0) {
    throw std::exception("[Scene]: empty scene");
  }

  if (id >= 0 && id <= models.size() - 1) {
    return models[id];
  }

  throw std::out_of_range("[Scene]: model index out of range");
}

int Scene::numOfModels() { return models.size(); }

void Scene::clear() { models.clear(); }
