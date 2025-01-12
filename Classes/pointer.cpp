#include "pointer.h"
#include "Parser.h"
#include <glm/gtc/type_ptr.hpp>

Pointer::Pointer(std::string path, glm::vec4 color) 
	: Figure(Parser::ParseObj(path)), color(color) {}

void Pointer::Render(int colorLoc)
{
	vao.Bind();

	glUniform4fv(colorLoc, 1, glm::value_ptr(color));
	glDrawElements(GL_TRIANGLES, indices_count, GL_UNSIGNED_INT, 0);

	vao.Unbind();
}