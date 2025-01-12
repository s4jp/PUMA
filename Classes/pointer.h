#pragma once
#include "figure.h"

class Pointer : public Figure
{
public:
	glm::vec4 color;

	Pointer(std::string path, glm::vec4 color);

	void Render(int colorLoc) override;
};