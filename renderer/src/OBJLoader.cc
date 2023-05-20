#include "OBJLoader.h"

MeshModel::MeshModel(std::vector<Eigen::Vector3f>& verts)
    : numVertices(verts.size()),
      vertices(verts),
      useNormals(false),
      useTexture(false) {}

MeshModel::MeshModel(const char* filePath, bool noSlash) {
  ObjLoader modelLoader;
  if (noSlash) {
    modelLoader.ParseObjNoSlash(filePath);
  } else {
    modelLoader.ParseObj(filePath);
  }
  numVertices = modelLoader.numVertices;
  int numRenderVertices = modelLoader.triangleVertices.size() / 3;
  int numFaces = numRenderVertices / 3;
  // std::cout << "Number of identical vertices: " << numVertices << std::endl;
  // std::cout << "Number of faces: " << numFaces << std::endl;

  std::vector<float>& verts = modelLoader.triangleVertices;
  std::vector<float>& tcs = modelLoader.textureValues;
  std::vector<float>& normals = modelLoader.faceNormals;

  useTexture = tcs.size() > 0 ? true : false;
  useNormals = normals.size() > 0 ? true : false;

  for (int i = 0; i < numRenderVertices; i++) {
    vertices.push_back(
        Eigen::Vector3f(verts[i * 3 + 0], verts[i * 3 + 1], verts[i * 3 + 2]));
    if (useTexture) {
      texCoords.push_back(Eigen::Vector2f(tcs[i * 2 + 0], tcs[i * 2 + 1]));
    }
    if (useNormals) {
      normalVecs.push_back(Eigen::Vector3f(
          normals[i * 3 + 0], normals[i * 3 + 1], normals[i * 3 + 2]));
    }
  }
}

void MeshModel::centerAligned() {
  if (vertices.empty()) {
    return;
  }
  int N = vertices.size();

  Eigen::VectorXd xCoords(N), yCoords(N), zCoords(N);
  for (int i = 0; i < N; i++) {
    xCoords(i) = vertices[i].x();
    yCoords(i) = vertices[i].y();
    zCoords(i) = vertices[i].z();
  }

  float midX = (xCoords.maxCoeff() + xCoords.minCoeff()) / 2.0;
  float midY = (yCoords.maxCoeff() + yCoords.minCoeff()) / 2.0;
  float midZ = (zCoords.maxCoeff() + zCoords.minCoeff()) / 2.0;

  Eigen::Affine3f centerAdjustment = Eigen::Affine3f::Identity();
  centerAdjustment.translate(Eigen::Vector3f(-midX, -midY, -midZ));

  for (int i = 0; i < N; i++) {
    vertices[i] = centerAdjustment * vertices[i];
  }
}

int MeshModel::getNumVertices() { return numVertices; }

std::vector<Eigen::Vector3f>& MeshModel::getVertices() { return vertices; }

std::vector<Eigen::Vector2f>& MeshModel::getTextureCoords() {
  return texCoords;
}

std::vector<Eigen::Vector3f>& MeshModel::getNormals() { return normalVecs; }

ObjLoader::ObjLoader() : numVertices(0) {}

void ObjLoader::ParseObj(const char* filePath) {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  std::string content;
  std::ifstream fileStream(filePath, std::ios::in);
  std::string line = "";

  bool withTexture = false;
  bool withNormal = false;

  int cnt = 0;
  while (!fileStream.eof()) {
    std::getline(fileStream, line);
    if (line.compare(0, 2, "v ") == 0) {
      std::stringstream ss(line.erase(0, 2));
      ss >> x >> y >> z;
      vertValues.push_back(x);
      vertValues.push_back(y);
      vertValues.push_back(z);
      numVertices++;
    }
    if (line.compare(0, 2, "vt") == 0) {
      withTexture = true;
      std::stringstream ss(line.erase(0, 2));
      ss >> x >> y;
      stValues.push_back(x);
      stValues.push_back(y);
    }
    if (line.compare(0, 2, "vn") == 0) {
      withNormal = true;
      std::stringstream ss(line.erase(0, 2));
      ss >> x >> y >> z;
      normalValues.push_back(x);
      normalValues.push_back(y);
      normalValues.push_back(z);
    }
    if (line.compare(0, 2, "f ") == 0) {
      std::string oneCorner, v, t, n;
      std::stringstream ss(line.erase(0, 2));
      for (int i = 0; i < 3; i++) {
        std::getline(ss, oneCorner, ' ');
        std::stringstream oneCornerSS(oneCorner);
        std::getline(oneCornerSS, v, '/');
        std::getline(oneCornerSS, t, '/');
        std::getline(oneCornerSS, n, '/');

        int vertIdx = (std::stoi(v) - 1) * 3;
        int tcIdx, normIdx;
        if (withTexture) {
          tcIdx = (std::stoi(t) - 1) * 2;
        }
        if (withNormal) {
          normIdx = (std::stoi(n) - 1) * 3;
        }

        triangleVertices.push_back(vertValues[vertIdx]);
        triangleVertices.push_back(vertValues[vertIdx + 1]);
        triangleVertices.push_back(vertValues[vertIdx + 2]);

        if (withTexture) {
          textureValues.push_back(stValues[tcIdx]);
          textureValues.push_back(stValues[tcIdx + 1]);
        }

        if (withNormal) {
          faceNormals.push_back(normalValues[normIdx]);
          faceNormals.push_back(normalValues[normIdx + 1]);
          faceNormals.push_back(normalValues[normIdx + 2]);
        }
      }
    }
  }

  fileStream.close();
}

void ObjLoader::ParseObjNoSlash(const char* filePath) {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  std::string content;
  std::ifstream fileStream(filePath, std::ios::in);
  std::string line = "";

  int cnt = 0;
  while (!fileStream.eof()) {
    std::getline(fileStream, line);
    if (line.compare(0, 2, "v ") == 0) {
      std::stringstream ss(line.erase(0, 1));
      ss >> x >> y >> z;
      vertValues.push_back(x);
      vertValues.push_back(y);
      vertValues.push_back(z);
      numVertices++;
    }

    if (line.compare(0, 3, "f  ") == 0) {
      std::string oneCorner, v;
      std::stringstream ss(line.erase(0, 3));
      for (int i = 0; i < 3; i++) {
        std::getline(ss, oneCorner, ' ');
        std::stringstream oneCornerSS(oneCorner);
        std::getline(oneCornerSS, v, ' ');

        int vertIdx = (std::stoi(v) - 1) * 3;

        triangleVertices.push_back(vertValues[vertIdx]);
        triangleVertices.push_back(vertValues[vertIdx + 1]);
        triangleVertices.push_back(vertValues[vertIdx + 2]);
      }
    }
  }

  fileStream.close();
}
