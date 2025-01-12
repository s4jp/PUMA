#pragma once
#define _USE_MATH_DEFINES
#include "VAO.h"
#include "VBO.h"
#include "EBO.h"
#include "helpers.h"

#include <glm/glm.hpp>
#include <tuple>
#include <string>

class Figure
{
protected:
  VAO vao;
  VBO vbo;
  EBO ebo;

  size_t indices_count;

public:
  virtual void Render(int colorLoc) = 0;

  void Delete() {
    vao.Delete();
    vbo.Delete();
    ebo.Delete();
  }

  Figure(std::tuple<std::vector<GLfloat>, std::vector<GLuint>> data) {
    indices_count = std::get<1>(data).size();

    vao.Bind();
    vbo = VBO(std::get<0>(data).data(), std::get<0>(data).size() * sizeof(GLfloat));
    ebo = EBO(std::get<1>(data).data(), std::get<1>(data).size() * sizeof(GLint));

    vao.LinkAttrib(vbo, 0, 3, GL_FLOAT, 0, (void *)0);
    vao.Unbind();
    vbo.Unbind();
    ebo.Unbind();
  }

  Figure(std::tuple<std::vector<VertexStruct>, std::vector<GLuint>> data) {
      indices_count = std::get<1>(data).size();

      vao.Bind();
      vbo = VBO(std::get<0>(data).data(), std::get<0>(data).size() * sizeof(VertexStruct));
      ebo = EBO(std::get<1>(data).data(), std::get<1>(data).size() * sizeof(GLint));

      vao.LinkAttrib(vbo, 0, 3, GL_FLOAT, sizeof(VertexStruct), (void*)0);
	  vao.LinkAttrib(vbo, 1, 3, GL_FLOAT, sizeof(VertexStruct), (void*)offsetof(VertexStruct, normal));
      vao.Unbind();
      vbo.Unbind();
      ebo.Unbind();
  }

  void RefreshBuffers(std::tuple<std::vector<GLfloat>, std::vector<GLuint>> data) {
      indices_count = std::get<1>(data).size();
	  vao.Bind();
      vbo.ReplaceBufferData(std::get<0>(data).data(), std::get<0>(data).size() * sizeof(GLfloat));
      ebo.ReplaceBufferData(std::get<1>(data).data(), std::get<1>(data).size() * sizeof(GLint));
	  vao.Unbind();
  }

  void RefreshBuffers(std::tuple<std::vector<VertexStruct>, std::vector<GLuint>> data) {
      indices_count = std::get<1>(data).size();
	  vao.Bind();
	  vbo.ReplaceBufferData(std::get<0>(data).data(), std::get<0>(data).size() * sizeof(VertexStruct));
	  ebo.ReplaceBufferData(std::get<1>(data).data(), std::get<1>(data).size() * sizeof(GLint));
	  vao.Unbind();
  }
};