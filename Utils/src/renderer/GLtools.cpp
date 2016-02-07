#include "Utils/renderer/GLtools.hpp"

#include <iostream>

namespace Utils {

GLuint buildProgram(const GLchar* vertexShaderSource, const GLchar* fragmentShaderSource) {
    // Vertex Shader Creation
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);

    // Specify Source Code
    glShaderSource(vertexShader, 1, &vertexShaderSource, 0);

    // Compile shader
    glCompileShader(vertexShader);

    // Check if compilation is terminated
    GLint compileStatus;
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &compileStatus);
    if(compileStatus == GL_FALSE) {

        GLint logLength;
        glGetShaderiv(vertexShader, GL_INFO_LOG_LENGTH, &logLength);

        char* log = new char[logLength];

        glGetShaderInfoLog(vertexShader, logLength, 0, log);
        std::cerr << "Vertex Shader error:" << log << std::endl;
        std::cerr << vertexShaderSource << std::endl;

        delete [] log;
        return 0;
    }

    // Fragment Shader Creation
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    // Specify Source Code
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, 0);

    // Compile shader
    glCompileShader(fragmentShader);

    // Check if compilation is terminated
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &compileStatus);
    if(compileStatus == GL_FALSE) {

        GLint logLength;
        glGetShaderiv(fragmentShader, GL_INFO_LOG_LENGTH, &logLength);

        char* log = new char[logLength];

        glGetShaderInfoLog(fragmentShader, logLength, 0, log);
        std::cerr << "Fragment Shader error:" << log << std::endl;
        std::cerr << fragmentShaderSource << std::endl;

        delete [] log;
        return 0;
    }

    GLuint program;

    // Program Creation
    program = glCreateProgram();

    // Link Shaders in the program
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Link
    glLinkProgram(program);

    GLint linkStatus;
    glGetProgramiv(program, GL_LINK_STATUS, &linkStatus);
    if(linkStatus == GL_FALSE) {

        GLint logLength;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);

        char* log = new char[logLength];

        glGetProgramInfoLog(program, logLength, 0, log);
        std::cerr << "Program link error:" << log << std::endl;

        delete [] log;
        return 0;
    }

    return program;
}

}
