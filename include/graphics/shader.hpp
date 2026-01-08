#ifndef SHADER_HPP
#define SHADER_HPP

#include <string>
#include <GL/glew.h>
#include <glm/glm.hpp>

class Shader {
    private:
        unsigned int id;

        unsigned int compileShader(unsigned int type, const char *src) const;

        void checkLinkError(unsigned int shaderProgram) const;

    public:
        Shader(const char* vertexCode, const char* fragmentCode);
        ~Shader();

        void use() const;

        void setMat4(std::string uniformName, glm::mat4 m);
};

#endif

