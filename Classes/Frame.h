#pragma once
#include "glm/glm.hpp"

class Frame
{
private:
	glm::vec3 x;
	glm::vec3 y;
	glm::vec3 z;
	glm::vec3 origin;

public:
	Frame();
	Frame(glm::vec3 x, glm::vec3 y, glm::vec3 z, glm::vec3 origin);
	Frame(glm::vec3 translation, glm::quat rotation);

	glm::vec3 GetX() const;
	glm::vec3 GetY() const;
	glm::vec3 GetZ() const;
	glm::vec3 GetOrigin() const;

	void Rotate(glm::quat rotation);
	void Translate(glm::vec3 translation);
	glm::mat4 GetMatrix() const;
};