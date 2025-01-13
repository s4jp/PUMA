#include "mesh.h"
#include "Parser.h"
#include <glm/gtc/type_ptr.hpp>

Mesh::Mesh(std::string path, glm::vec4 color)
	: Figure(Parser::ParseObj(path)), color(color) {}

void Mesh::Render(int colorLoc)
{
	vao.Bind();

	glUniform4fv(colorLoc, 1, glm::value_ptr(color));
	glDrawElements(GL_TRIANGLES, indices_count, GL_UNSIGNED_INT, 0);

	vao.Unbind();
}