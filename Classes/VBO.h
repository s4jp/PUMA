#ifndef VBO_CLASS_H
#define VBO_CLASS_H

#include "glad/glad.h"
#include <glm/glm.hpp>
#include "VertexStruct.h"

class VBO
{
public:
	GLuint ID;
	VBO() {};
	VBO(GLfloat* vertices, GLsizeiptr size);
	VBO(VertexStruct* vertices, GLsizeiptr size);

	void Bind();
	void Unbind();
    void Delete();
    void ReplaceBufferData(GLfloat *vertices, GLsizeiptr size);
	void ReplaceBufferData(VertexStruct* vertices, GLsizeiptr size);
};
#endif