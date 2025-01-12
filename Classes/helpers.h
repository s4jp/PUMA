#pragma once

#include "glm/glm.hpp"
#include "Camera.h"
#include <vector>
#include <string>
#include <tuple>

namespace CAD 
{
	glm::mat4 translate(glm::mat4 matrix, glm::vec3 vector);
	glm::mat4 createXrotationMatrix(float angle);
    glm::mat4 createYrotationMatrix(float angle);
    glm::mat4 createZrotationMatrix(float angle);
    glm::mat4 rotate(glm::mat4 matrix, glm::vec3 angle);
    glm::mat4 projection(float fov, float ratio, float near, float far);
    glm::mat4 scaling(glm::mat4 matrix, glm::vec3 scale);
    void printVector(glm::vec3 vec);
    glm::mat4 lookAt(glm::vec3 position, glm::vec3 target, glm::vec3 up);
    void printMatrix(glm::mat4 mat);
    float angleBetweenVectors(glm::vec3 u, glm::vec3 v);
    std::string printPosition(glm::vec3 pos, std::string name = "");
 }