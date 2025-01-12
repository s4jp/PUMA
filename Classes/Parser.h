#pragma once

#include <tuple>
#include <VertexStruct.h>
#include <vector>
#include <glad/glad.h>
#include <string>
#include <fstream>
#include <sstream>

class Parser
{
public: 

	static std::tuple<std::vector<VertexStruct>, std::vector<GLuint>> ParseObj(std::string path)
	{
		std::vector<VertexStruct> vertices;
		std::vector<GLuint> indices;

		std::ifstream file(path);
		if (!file.is_open())
		{
			throw std::runtime_error("Failed to open file");
		}

		std::string line;
		int lineCounter = 0;
		std::vector<glm::vec3> positions;
		std::vector<glm::vec3> normals;

		while (std::getline(file, line)) {
			lineCounter++;

			std::stringstream ss(line);
			std::vector<std::string> substrings;
			std::string word;

			while (std::getline(ss, word, ' ')) {
				substrings.push_back(word);
			}

			if (substrings.size() != 4) {
				throw std::runtime_error("Line #" + std::to_string(lineCounter) + " does not contain exactly 4 substrings");
			}

			const std::string& firstSubstring = substrings[0];
			if (firstSubstring == "v") {
				glm::vec3 position;
				position.x = std::stof(substrings[1]);
				position.y = std::stof(substrings[2]);
				position.z = std::stof(substrings[3]);
				positions.push_back(position);
			}
			else if (firstSubstring == "vn") {
				glm::vec3 normal;
				normal.x = std::stof(substrings[1]);
				normal.y = std::stof(substrings[2]);
				normal.z = std::stof(substrings[3]);
				normals.push_back(normal);
			}
			else if (firstSubstring == "f") {
				for (int i = 1; i <= 3; i++) {

					std::stringstream faceStream(substrings[i]);
					std::vector<std::string> faceSubstrings;
					std::string faceWord;

					while (std::getline(faceStream, faceWord, '/')) {
						faceSubstrings.push_back(faceWord);
					}

					if (faceSubstrings.size() != 3) {
						throw std::runtime_error("Invalid face substring in line#" + std::to_string(lineCounter));
					}

					VertexStruct vertex;
					vertex.position = positions[std::stoi(faceSubstrings[0]) - 1];
					vertex.normal = normals[std::stoi(faceSubstrings[2]) - 1];
					vertices.push_back(vertex);
					indices.push_back(indices.size());
				}
			}
			else {
				throw std::runtime_error("Invalid first substring in line#" + std::to_string(lineCounter));
			}
		}

		file.close();
		return std::make_tuple(vertices, indices);
	};
};