#pragma once
#include "figure.h"

class Cylinder : public Figure
{
public:
	glm::vec4 color;

	Cylinder(std::string path, glm::vec4 color);

	void Render(int colorLoc) override;
};