#include "../include/shaderBuilder.h"

ShaderBuilder* ShaderBuilder::pInstance = nullptr;

ShaderBuilder* ShaderBuilder::getBuilder() {
  if (pInstance == nullptr) {
    pInstance = new ShaderBuilder();
  }
  return pInstance;
}

ShaderBuilder::~ShaderBuilder() {
  if (pInstance != nullptr) {
    delete pInstance;
    pInstance = nullptr;
  }
}

void ShaderBuilder::compile(Shader& s, ShaderType t) {
  const char* code = s.code.c_str();

  GLuint shaderID;

  switch (t) {
    case ShaderType::VERTEX:
      shaderID = glCreateShader(GL_VERTEX_SHADER);
      break;
    case ShaderType::FRAGMENT:
      shaderID = glCreateShader(GL_FRAGMENT_SHADER);
      break;
    default:
      std::cerr << "[ShaderBuilder] Error: shader type is unsupported"
                << std::endl;
      return;
  }
  glShaderSource(shaderID, 1, &code, NULL);
  glCompileShader(shaderID);

  int success;
  char infoLog[512];
  glGetShaderiv(shaderID, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(shaderID, 512, NULL, infoLog);
    std::cout << "[ShaderBuilder] Error: shader compilation failed\n"
              << infoLog << std::endl;
    return;
  }

  s.ID = shaderID;
  s.compiled = true;
}

Shader readShaderFromSource(const char* path) {
  std::string code;
  std::ifstream shaderFile(path);
  if (!shaderFile.is_open()) {
    fprintf(stderr, "[OPENGL RENDERER]: Open File %s Failed\n", path);
    exit(-1);
  }

  // read the string from the shaderFile
  std::stringstream ss;
  ss << shaderFile.rdbuf();
  code = ss.str();

  shaderFile.close();

  return Shader{path, code};
}