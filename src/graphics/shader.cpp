#include <GL/glew.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include "graphics/shader.hpp"

Shader::Shader(const char* vertexCode, const char* fragmentCode) {

    unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexCode);
    unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentCode);

    id = glCreateProgram();
    glAttachShader(id, vertexShader);
    glAttachShader(id, fragmentShader);
    glLinkProgram(id);
    
    checkLinkError(id);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

Shader::~Shader() {
    glDeleteProgram(id);
}

void Shader::use() const {
    glUseProgram(id);
}

void Shader::setMat4(std::string uniformName, glm::mat4 m) {
    glUniformMatrix4fv(glGetUniformLocation(id, uniformName.c_str()), 1, GL_FALSE, &m[0][0]);
}

unsigned int Shader::compileShader(unsigned int type, const char *src) const {
    unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "FAILED TO COMPILE SHADER: " << infoLog << std::endl;
    }

    return shader;
}

void Shader::checkLinkError(unsigned int shaderProgram) const {
    GLint success;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram, 512, nullptr, infoLog);
        std::cerr << "Shader linking error: " << infoLog << std::endl;
    }
}

