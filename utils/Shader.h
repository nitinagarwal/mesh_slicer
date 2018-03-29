/* This is a shader class which reads both vertex and fragment shader txtfile */
#ifndef SHADER_H
#define SHADER_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <GL/glew.h> // include glew to get all the required OpenGL headers
#include <glm/glm.hpp>

class Shader
{

    private:
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;

        /* Defining stringstream */
        std::stringstream vShaderStream, fShaderStream;

        /* Defining c_str */
        const GLchar* vShaderCode ;
        const GLchar* fShaderCode ;

        /* Defining vertex and fragment shaders */
        GLuint vertex, fragment;

        /* to check compile and linking errors */
        void checkCompileErrors(GLuint shader, std::string type);

    public:
        /* The program ID */
        GLuint Program;

        /* Constructor reads and builds the shader */
        Shader(const GLchar* vertexPath, const GLchar* fragmentPath);

        /* Use the program */
        void Use();


    // utility uniform functions
    void setBool(const std::string &name, bool value) const;
    // ------------------------------------------------------------------------
    void setInt(const std::string &name, int value) const;
    // ------------------------------------------------------------------------
    void setFloat(const std::string &name, float value) const;
    // ------------------------------------------------------------------------
    void setVec2(const std::string &name, const glm::vec2 &value) const;
    void setVec2(const std::string &name, float x, float y) const;
    // ------------------------------------------------------------------------
    void setVec3(const std::string &name, const glm::vec3 &value) const;
    void setVec3(const std::string &name, float x, float y, float z) const;
    // ------------------------------------------------------------------------
    void setVec4(const std::string &name, const glm::vec4 &value) const;
    void setVec4(const std::string &name, float x, float y, float z, float w) ;
    // ------------------------------------------------------------------------
    void setMat2(const std::string &name, const glm::mat2 &mat) const;
    // ------------------------------------------------------------------------
    void setMat3(const std::string &name, const glm::mat3 &mat) const;
    // ------------------------------------------------------------------------
    void setMat4(const std::string &name, const glm::mat4 &mat) const;

};

#endif
