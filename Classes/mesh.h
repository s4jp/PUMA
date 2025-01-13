#pragma once
#include "figure.h"

class Mesh : public Figure
{
public:
	glm::vec4 color;

	Mesh(std::string path, glm::vec4 color);

	void Render(int colorLoc) override;
};