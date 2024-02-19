#include "../include/OpenGLRenderer.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
  glViewport(0, 0, width, height);
}

float toRadians(float degree) { return degree * M_PI / 180.0; }

OpenGLRenderer::OpenGLRenderer(GLFWwindow* window, int width, int height)
    : context(window), windowWidth(width), windowHeight(height) {}

OpenGLRenderer::~OpenGLRenderer() {
  // glfwTerminate();
}

void OpenGLRenderer::addShader(const char* path, ShaderType t, int id) {
  Shader res = readShaderFromSource(path);
  if (id < 0) {
    throw std::out_of_range("[OPENGL RENDERER]: Index should not be negative");
  }

  addShader(t, res, id);
}

void OpenGLRenderer::addShader(ShaderType t, Shader s, int id) {
  if (shaders.empty()) {
    std::unordered_map<ShaderType, Shader> m;
    m.insert(std::make_pair(t, s));
    shaders.push_back(m);
  } else {
    if (id <= shaders.size() - 1) {
      shaders[id].insert(std::make_pair(t, s));
    } else {
      std::unordered_map<ShaderType, Shader> m;
      m.insert(std::make_pair(t, s));
      shaders.push_back(m);
    }
  }
}

void OpenGLRenderer::clearShader() { shaders.clear(); }

void OpenGLRenderer::bindWindow(GLFWwindow* window) { context = window; }

std::vector<GLubyte> OpenGLRenderer::snapshot(Scene& scene, Camera& camera) {
  glViewport(0, 0, windowWidth, windowHeight);
  glfwSetFramebufferSizeCallback(context, framebuffer_size_callback);

  int N = scene.numOfModels();
  std::vector<GLuint> linkedPrograms(N);
  int success = true;
  char infoLog[512];
  for (int i = 0; i < N; i++) {
    linkedPrograms[i] = glCreateProgram();

    ShaderBuilder* sb = ShaderBuilder::getBuilder();
    if (shaders[i].find(ShaderType::VERTEX) == shaders[i].end()) {
      throw std::exception("[OPENGL RENDERER]: vertex shader not found");
    }
    Shader& vs = shaders[i][ShaderType::VERTEX];
    sb->compile(vs, ShaderType::VERTEX);
    glAttachShader(linkedPrograms[i], vs.ID);
    if (shaders[i].find(ShaderType::FRAGMENT) == shaders[i].end()) {
      throw std::exception("[OPENGL RENDERER]: fragment shader not found");
    }
    Shader& fs = shaders[i][ShaderType::FRAGMENT];
    sb->compile(fs, ShaderType::FRAGMENT);
    glAttachShader(linkedPrograms[i], fs.ID);

    glLinkProgram(linkedPrograms[i]);

    glGetShaderiv(linkedPrograms[i], GL_LINK_STATUS, &success);
    if (!success) {
      glGetShaderInfoLog(linkedPrograms[i], 512, NULL, infoLog);
      std::cout << "[OPENGL RENDERER] Error: shader program linking failed\n"
                << infoLog << std::endl;
    }

    glDeleteShader(shaders[i][ShaderType::VERTEX].ID);
    glDeleteShader(shaders[i][ShaderType::FRAGMENT].ID);
  }

  int vaoSize = scene.numOfModels();
  int vboSize = 3 * scene.numOfModels();  // vertex + normal + texture
  VAO.clear();
  VBO.clear();
  VAO.resize(vaoSize);
  VBO.resize(vboSize);

  glGenVertexArrays(vaoSize, VAO.data());
  glGenBuffers(vboSize, VBO.data());

  for (int i = 0; i < vaoSize; i++) {
    MeshModel& m = scene.get(i);

    std::vector<float> vertexVals;
    std::vector<Eigen::Vector3f>& v = m.getVertices();
    for (int i = 0; i < v.size(); i++) {
      vertexVals.push_back(v[i].x());
      vertexVals.push_back(v[i].y());
      vertexVals.push_back(v[i].z());
    }

    glBindVertexArray(VAO[i]);

    glBindBuffer(GL_ARRAY_BUFFER, VBO[3 * i]);
    glBufferData(GL_ARRAY_BUFFER, vertexVals.size() * sizeof(float),
                 vertexVals.data(), GL_STATIC_DRAW);  // GL_STATIC_COPY
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void*)0);
    glEnableVertexAttribArray(0);

    if (m.useTexture) {
      std::vector<float> textureVals;
      for (const Eigen::Vector2f& tc : m.getTextureCoords()) {
        textureVals.push_back(tc.x());
        textureVals.push_back(tc.y());
      }
      glBindBuffer(GL_ARRAY_BUFFER, VBO[3 * i + 1]);
      glBufferData(GL_ARRAY_BUFFER, textureVals.size() * sizeof(float),
                   textureVals.data(), GL_STATIC_DRAW);
      glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float),
                            (void*)0);
      glEnableVertexAttribArray(1);

      glActiveTexture(GL_TEXTURE0);
    }

    if (m.useNormals) {
      std::vector<float> normalVals;
      for (const Eigen::Vector3f& n : m.getNormals()) {
        normalVals.push_back(n.x());
        normalVals.push_back(n.y());
        normalVals.push_back(n.z());
      }
      glBindBuffer(GL_ARRAY_BUFFER, VBO[3 * i + 2]);
      glBufferData(GL_ARRAY_BUFFER, normalVals.size() * sizeof(float),
                   normalVals.data(), GL_STATIC_DRAW);
      glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                            (void*)0);
      glEnableVertexAttribArray(2);
    }
  }

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_STENCIL_TEST);

  std::vector<GLubyte> stencilBufferData(windowWidth * windowHeight * 2,
                                         0);  // double buffering

  Eigen::Vector3d directionOfView(1.0, 0.0, 1.0);

  // Stencil buffer settings
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  glClearColor(0.1, 0.5, 0.5, 1.0);
  glClearStencil(0);
  glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
  glStencilFunc(GL_ALWAYS, 1, 0xFF);
  glStencilMask(0xFF);

  float rotationDegreeX = 0.0;
  float rotationDegreeY = 0.0;
  float rotationDegreeZ = 0.0;

  Eigen::Affine3f modelTransform = Eigen::Affine3f::Identity();
  modelTransform.rotate(Eigen::AngleAxisf(toRadians(rotationDegreeX),
                                          Eigen::Vector3f(-1.0, 0.0, 0.0)));
  modelTransform.rotate(Eigen::AngleAxisf(toRadians(rotationDegreeY),
                                          Eigen::Vector3f(0.0, -1.0, 0.0)));
  modelTransform.rotate(Eigen::AngleAxisf(toRadians(rotationDegreeZ),
                                          Eigen::Vector3f(0.0, 0.0, -1.0)));

  Eigen::Matrix4f modelMatrix = modelTransform.matrix();
  Eigen::Matrix4f viewMatrix = camera.getViewMatrix();

  Eigen::Matrix4f projectMatrix = camera.getProjectMatrix();
  Eigen::Matrix4f mvp = projectMatrix * viewMatrix * modelMatrix;
  Eigen::Matrix4f res = mvp.eval();

  // use shader program, load GLSL code
  for (int id = 0; id < scene.numOfModels(); id++) {
    glBindBuffer(GL_ARRAY_BUFFER, VBO[id * 3]);
    int verticesCnt = scene.get(id).getVertices().size();
    int mvpLoc = glGetUniformLocation(linkedPrograms[id], "mvp");
    glUseProgram(linkedPrograms[id]);
    glBindVertexArray(VAO[id]);
    glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, res.data());
    glDrawArrays(GL_TRIANGLES, 0, verticesCnt);
  }

  glfwSwapBuffers(context);
  glfwPollEvents();

  // https://community.khronos.org/t/how-to-read-back-stencil-buffer/43640
  glReadPixels(0, 0, windowWidth, windowHeight, GL_STENCIL_INDEX,
               GL_UNSIGNED_BYTE, stencilBufferData.data());

  return stencilBufferData;
}
