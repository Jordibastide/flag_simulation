#pragma once

#include <GL/glew.h>

#define GL_STRINGIFY(s) #s

namespace Utils {

GLuint buildProgram(const GLchar* vertexShaderSrc, const GLchar* fragmentShaderSrc);

}
