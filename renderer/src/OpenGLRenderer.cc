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
  // 读取文件
  std::string code;
  std::ifstream shaderFile(path);
  if (!shaderFile.is_open()) {
    fprintf(stderr, "[OPENGL RENDERER]: Open File %s Failed\n", path);
    exit(-1);
  }

  std::string s = "";
  while (std::getline(shaderFile, s)) {
    code = code + s + "\n";
  }
  shaderFile.close();

  Shader res{path, code};
  if (id < 0) {
    throw std::out_of_range("[OPENGL RENDERER]: Index should not be negative");
  }

  if (shaders.empty()) {
    std::unordered_map<ShaderType, Shader> m;
    m.insert(std::make_pair(t, res));
    shaders.push_back(m);
  } else {
    if (id <= shaders.size() - 1) {
      shaders[id].insert(std::make_pair(t, res));
    } else {
      std::unordered_map<ShaderType, Shader> m;
      m.insert(std::make_pair(t, res));
      shaders.push_back(m);
    }
  }
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

void OpenGLRenderer::render(Scene& scene, Camera& camera) {
  glViewport(0, 0, windowWidth, windowHeight);
  glfwSetFramebufferSizeCallback(context, framebuffer_size_callback);

  // 加载着色器
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
  int vboSize = 3 * scene.numOfModels();
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
      // glBindTexture(GL_TEXTURE_2D, )
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
                                         0);  // 双缓冲

  Eigen::Vector3d directionOfView(1.0, 0.0, 1.0);
  // 直角坐标系转球坐标系

  while (!glfwWindowShouldClose(context)) {
    // 渲染到 Stencil buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0.1, 0.5, 0.5, 1.0);
    glClearStencil(0);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glStencilFunc(
        GL_ALWAYS, 1,
        0xFF);  // 只要一个片段的模板值等于(GL_EQUAL)参考值1，片段将会通过测试并被绘制，否则会被丢弃。
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

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // 启动着色器程序，加载GLSL代码
    for (int id = 0; id < scene.numOfModels(); id++) {
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
  }

  for (int i = windowHeight - 1; i >= 0; i--) {
    for (int j = 0; j < windowWidth; j++) {
      printf("%hhu", stencilBufferData[i * windowWidth + j]);
    }
    printf("\n");
  }
}

std::vector<GLubyte> OpenGLRenderer::snapshot(Scene& scene, Camera& camera) {
  glViewport(0, 0, windowWidth, windowHeight);
  glfwSetFramebufferSizeCallback(context, framebuffer_size_callback);

  // 加载着色器
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
      // glBindTexture(GL_TEXTURE_2D, )
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
                                         0);  // 双缓冲

  Eigen::Vector3d directionOfView(1.0, 0.0, 1.0);

  // 渲染到 Stencil buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  glClearColor(0.1, 0.5, 0.5, 1.0);
  glClearStencil(0);
  glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
  glStencilFunc(
      GL_ALWAYS, 1,
      0xFF);  // 只要一个片段的模板值等于(GL_EQUAL)参考值1，片段将会通过测试并被绘制，否则会被丢弃。
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

  // 启动着色器程序，加载GLSL代码
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

std::vector<GLubyte> OpenGLRenderer::snapshotV2(Scene& scene, Camera& camera,
                                                int modelIdx, int shaderIdx) {
  glViewport(0, 0, windowWidth, windowHeight);

  // 加载着色器
  GLuint linkedProgram;
  int success = true;
  char infoLog[512];
  linkedProgram = glCreateProgram();

  ShaderBuilder* sb = ShaderBuilder::getBuilder();
  if (shaders[shaderIdx].find(ShaderType::VERTEX) == shaders[shaderIdx].end()) {
    throw std::exception("[OPENGL RENDERER]: vertex shader not found");
  }
  Shader& vs = shaders[shaderIdx][ShaderType::VERTEX];
  if (vs.ID == -1) {
    sb->compile(vs, ShaderType::VERTEX);
  }
  glAttachShader(linkedProgram, vs.ID);
  if (shaders[shaderIdx].find(ShaderType::FRAGMENT) ==
      shaders[shaderIdx].end()) {
    throw std::exception("[OPENGL RENDERER]: fragment shader not found");
  }
  Shader& fs = shaders[shaderIdx][ShaderType::FRAGMENT];
  if (fs.ID == -1) {
    sb->compile(fs, ShaderType::FRAGMENT);
  }
  glAttachShader(linkedProgram, fs.ID);

  glLinkProgram(linkedProgram);

  glGetShaderiv(linkedProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(linkedProgram, 512, NULL, infoLog);
    std::cout << "[OPENGL RENDERER] Error: shader program linking failed\n"
              << infoLog << std::endl;
  }

  int vboSize = 3;
  VAO.clear();
  VBO.clear();
  VAO.resize(1);
  VBO.resize(vboSize);

  glGenVertexArrays(1, VAO.data());
  glGenBuffers(vboSize, VBO.data());

  MeshModel& m = scene.get(modelIdx);

  std::vector<float> vertexVals;
  std::vector<Eigen::Vector3f>& v = m.getVertices();
  for (int i = 0; i < v.size(); i++) {
    vertexVals.push_back(v[i].x());
    vertexVals.push_back(v[i].y());
    vertexVals.push_back(v[i].z());
  }

  glBindVertexArray(VAO[0]);

  glBindBuffer(GL_ARRAY_BUFFER, VBO[3 * modelIdx]);
  glBufferData(GL_ARRAY_BUFFER, vertexVals.size() * sizeof(float),
               vertexVals.data(), GL_STATIC_DRAW);  // GL_STATIC_COPY
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_STENCIL_TEST);

  std::vector<GLubyte> stencilBufferData(windowWidth * windowHeight * 2,
                                         0);  // 双缓冲

  Eigen::Vector3d directionOfView(1.0, 0.0, 1.0);
  // 直角坐标系转球坐标系

  // 渲染到 Stencil buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  glClearColor(0.1, 0.5, 0.5, 1.0);
  glClearStencil(0);
  glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
  glStencilFunc(
      GL_ALWAYS, 1,
      0xFF);  // 只要一个片段的模板值等于(GL_EQUAL)参考值1，片段将会通过测试并被绘制，否则会被丢弃。
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

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  // 启动着色器程序，加载GLSL代码

  int verticesCnt = scene.get(modelIdx).getVertices().size();
  int mvpLoc = glGetUniformLocation(linkedProgram, "mvp");
  glUseProgram(linkedProgram);
  glBindVertexArray(VAO[0]);
  glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, res.data());

  glDrawArrays(GL_TRIANGLES, 0, verticesCnt);
  glfwSwapBuffers(context);
  glfwPollEvents();
  // https://community.khronos.org/t/how-to-read-back-stencil-buffer/43640
  glReadPixels(0, 0, windowWidth, windowHeight, GL_STENCIL_INDEX,
               GL_UNSIGNED_BYTE, stencilBufferData.data());

  return stencilBufferData;
}

// Reference:
// https://learnopengl.com/code_viewer_gh.php?code=src/4.advanced_opengl/10.1.instancing_quads/instancing_quads.cpp
std::vector<GLubyte> OpenGLRenderer::snapshotInstanced(
    Scene& scene, std::vector<Camera>& cameraArray,
    std::vector<Eigen::Vector2f>& offsets, int modelIdx, int shaderIdx,
    float scaleFactor) {
  const int instanceCnt = cameraArray.size();
  glViewport(0, 0, windowWidth, windowHeight);
  glfwSetFramebufferSizeCallback(context, framebuffer_size_callback);

  // 加载着色器
  GLuint linkedProgram;
  int success = true;
  char infoLog[512];
  linkedProgram = glCreateProgram();

  ShaderBuilder* sb = ShaderBuilder::getBuilder();
  if (shaders[shaderIdx].find(ShaderType::VERTEX) == shaders[shaderIdx].end()) {
    throw std::exception("[OPENGL RENDERER]: vertex shader not found");
  }
  Shader& vs = shaders[shaderIdx][ShaderType::VERTEX];
  if (vs.compiled == false) {
    sb->compile(vs, ShaderType::VERTEX);
  }
  glAttachShader(linkedProgram, vs.ID);
  if (shaders[shaderIdx].find(ShaderType::FRAGMENT) ==
      shaders[shaderIdx].end()) {
    throw std::exception("[OPENGL RENDERER]: fragment shader not found");
  }
  Shader& fs = shaders[shaderIdx][ShaderType::FRAGMENT];
  if (fs.compiled == false) {
    sb->compile(fs, ShaderType::FRAGMENT);
  }
  glAttachShader(linkedProgram, fs.ID);

  glLinkProgram(linkedProgram);

  glGetShaderiv(linkedProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(linkedProgram, 512, NULL, infoLog);
    std::cout << "[OPENGL RENDERER] Error: shader program linking failed\n"
              << infoLog << std::endl;
  }

  VAO.clear();
  VBO.clear();
  VAO.resize(1);
  VBO.resize(1);

  glGenVertexArrays(1, VAO.data());
  glGenBuffers(1, VBO.data());

  MeshModel& m = scene.get(modelIdx);

  std::vector<float> vertexVals;
  std::vector<Eigen::Vector3f>& v = m.getVertices();

  for (int i = 0; i < v.size(); i++) {
    vertexVals.push_back(v[i].x());
    vertexVals.push_back(v[i].y());
    vertexVals.push_back(v[i].z());
  }

  glBindVertexArray(VAO[0]);

  glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
  glBufferData(GL_ARRAY_BUFFER, vertexVals.size() * sizeof(float),
               vertexVals.data(), GL_STATIC_DRAW);  // GL_STATIC_COPY
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_STENCIL_TEST);

  std::vector<GLubyte> stencilBufferData(windowWidth * windowHeight * 2,
                                         0);  // 双缓冲

  Eigen::Vector3d directionOfView(1.0, 0.0, 1.0);

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

  std::vector<Eigen::Matrix4f> mvpArray(instanceCnt);
  // std::vector<float> mvpResultArray(instanceCnt * 4 * 4);

  for (int i = 0; i < instanceCnt; i++) {
    Eigen::Matrix4f viewMatrix = cameraArray[i].getViewMatrix();
    Eigen::Matrix4f projectMatrix = cameraArray[i].getProjectMatrix();
    // Eigen::Matrix4f mvp = projectMatrix * viewMatrix * modelMatrix; // 使用
    // glm 的写法
    Eigen::Matrix4f mvp = projectMatrix.transpose() * viewMatrix.transpose() *
                          modelMatrix.transpose();  // 使用 Eigen 的写法
    Eigen::Matrix4f res = mvp.eval();
    mvpArray[i] =
        res.transpose();  // Eigen 本身按列存，为了导入buffer，需要改为按行存
    // mvpArray[i] = res; // glm 本身按行存，不需要转置
  }

  // store instance data in an array buffer
  GLuint instanceVBOForMVP;
  glGenBuffers(1, &instanceVBOForMVP);
  glBindBuffer(GL_ARRAY_BUFFER, instanceVBOForMVP);
  glBufferData(GL_ARRAY_BUFFER, instanceCnt * sizeof(Eigen::Matrix4f),
               mvpArray.data(), GL_STATIC_DRAW);
  // glBufferData(GL_ARRAY_BUFFER, sizeof(float) * mvpResultArray.size(),
  // mvpResultArray.data(), GL_STATIC_DRAW);

  glBindVertexArray(VAO[0]);

  // set attribute pointers for matrix (4 times vec4)
  glEnableVertexAttribArray(3);
  glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Eigen::Matrix4f),
                        (void*)0);
  glEnableVertexAttribArray(4);
  glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(Eigen::Matrix4f),
                        (void*)(sizeof(Eigen::Vector4f)));
  glEnableVertexAttribArray(5);
  glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(Eigen::Matrix4f),
                        (void*)(2 * sizeof(Eigen::Vector4f)));
  glEnableVertexAttribArray(6);
  glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(Eigen::Matrix4f),
                        (void*)(3 * sizeof(Eigen::Vector4f)));

  glVertexAttribDivisor(3, 1);
  glVertexAttribDivisor(4, 1);
  glVertexAttribDivisor(5, 1);
  glVertexAttribDivisor(6, 1);

  GLuint instanceVBOForOffsets;
  glGenBuffers(1, &instanceVBOForOffsets);
  glBindBuffer(GL_ARRAY_BUFFER, instanceVBOForOffsets);
  glBufferData(GL_ARRAY_BUFFER, instanceCnt * sizeof(Eigen::Vector2f),
               offsets.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);  // reset

  glEnableVertexAttribArray(2);  // layout=2
  glBindBuffer(GL_ARRAY_BUFFER, instanceVBOForOffsets);
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector2f),
                        (void*)0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glVertexAttribDivisor(2, 1);

  // 渲染到 Stencil buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  glClearColor(0.1, 0.5, 0.5, 1.0);
  glClearStencil(0);
  glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
  glStencilFunc(
      GL_ALWAYS, 1,
      0xFF);  // 只要一个片段的模板值等于(GL_EQUAL)参考值1，片段将会通过测试并被绘制，否则会被丢弃。
  glStencilMask(0xFF);

  int verticesCnt = scene.get(modelIdx).getVertices().size();
  glUseProgram(linkedProgram);

  glUniform1f(glGetUniformLocation(linkedProgram, "scaleFactor"), scaleFactor);

  glBindVertexArray(VAO[0]);

  glDrawArraysInstanced(GL_TRIANGLES, 0, verticesCnt, instanceCnt);
  glBindVertexArray(0);

  glfwSwapBuffers(context);
  glfwPollEvents();

  // https://community.khronos.org/t/how-to-read-back-stencil-buffer/43640
  glReadPixels(0, 0, windowWidth, windowHeight, GL_STENCIL_INDEX,
               GL_UNSIGNED_BYTE, stencilBufferData.data());

  return stencilBufferData;
}
