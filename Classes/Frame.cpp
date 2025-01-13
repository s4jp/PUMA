#include "Frame.h"
#include <glm/gtx/quaternion.hpp>

Frame::Frame()
{
	x = glm::vec3(1.f, 0.f, 0.f);
	y = glm::vec3(0.f, 1.f, 0.f);
	z = glm::vec3(0.f, 0.f, 1.f);
	origin = glm::vec3(0.f);
}

Frame::Frame(glm::vec3 x, glm::vec3 y, glm::vec3 z, glm::vec3 origin)
{
	this->x = x;
	this->y = y;
	this->z = z;
	this->origin = origin;
}

Frame::Frame(glm::vec3 translation, glm::quat rotation)
{
	this->x = glm::vec3(1.f, 0.f, 0.f);
	this->y = glm::vec3(0.f, 1.f, 0.f);
	this->z = glm::vec3(0.f, 0.f, 1.f);
	this->origin = translation;
	this->Rotate(rotation);
}

Frame::Frame(const Frame& frame)
{
	this->x = frame.x;
	this->y = frame.y;
	this->z = frame.z;
	this->origin = frame.origin;
}

glm::vec3 Frame::GetX() const
{
	return x;
}

glm::vec3 Frame::GetY() const
{
	return y;
}

glm::vec3 Frame::GetZ() const
{
	return z;
}

glm::vec3 Frame::GetOrigin() const
{
	return origin;
}

void Frame::Rotate(glm::quat rotation)
{
	x = glm::normalize(rotation * x);
	y = glm::normalize(rotation * y);
	z = glm::normalize(rotation * z);
}

void Frame::Translate(glm::vec3 translation)
{
	origin += translation;
}

glm::mat4 Frame::GetMatrix() const
{
	glm::mat4 matrix(0.f);
	matrix[0] = glm::vec4(x, 0.f);
	matrix[1] = glm::vec4(y, 0.f);
	matrix[2] = glm::vec4(z, 0.f);
	matrix[3] = glm::vec4(origin, 1.f);
	return matrix;
}

glm::quat Frame::GetRotation() const
{
	return glm::quat_cast(glm::mat3(x, y, z));
}
