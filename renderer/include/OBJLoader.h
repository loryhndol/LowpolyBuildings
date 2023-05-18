#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class MeshModel {
 private:
  int numVertices;  // Identical. Not the number in the buffer
  std::vector<Eigen::Vector3f> vertices;
  std::vector<Eigen::Vector2f> texCoords;
  std::vector<Eigen::Vector3f> normalVecs;

 public:
  MeshModel(std::vector<Eigen::Vector3f>& verts);
  MeshModel(const char* filePath, bool noSlash = false);
  int getNumVertices();
  std::vector<Eigen::Vector3f>& getVertices();
  std::vector<Eigen::Vector2f>& getTextureCoords();
  std::vector<Eigen::Vector3f>& getNormals();
  bool useTexture;
  bool useNormals;
  void centerAligned();
};

class ObjLoader {
  friend class MeshModel;

 private:
  int numVertices;
  std::vector<float> vertValues;
  std::vector<float> triangleVertices;
  std::vector<float> textureValues;
  std::vector<float> stValues;
  std::vector<float> normalValues;
  std::vector<float> faceNormals;

 public:
  ObjLoader();
  void ParseObj(const char* filePath);
  void ParseObjNoSlash(const char* filePath);
};
