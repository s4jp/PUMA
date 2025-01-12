#pragma once

#include "glm/glm.hpp"

#include <atomic>
#include <mutex>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>

static int dt = 10;		// in milliseconds

glm::mat4 createRotationMatrixFromEulerAngles(const glm::vec3& eulerAngles) {
	glm::vec3 radians = glm::radians(eulerAngles);

	glm::mat4 rotationX = glm::rotate(glm::mat4(1.0f), radians.x, glm::vec3(1.0f, 0.0f, 0.0f));
	glm::mat4 rotationY = glm::rotate(glm::mat4(1.0f), radians.y, glm::vec3(0.0f, 1.0f, 0.0f));
	glm::mat4 rotationZ = glm::rotate(glm::mat4(1.0f), radians.z, glm::vec3(0.0f, 0.0f, 1.0f));

	glm::mat4 rotationMatrix = rotationZ * rotationY * rotationX;

	return rotationMatrix;
}

void normalizeTwoEulerAngles(glm::vec3& ea1, glm::vec3& ea2) {
	while (abs(ea1.x - ea2.x) > 180.f) ea2.x += (-1) * copysign(1.0, ea2.x) * 360.f;
	while (abs(ea1.y - ea2.y) > 180.f) ea2.y += (-1) * copysign(1.0, ea2.y) * 360.f;
	while (abs(ea1.z - ea2.z) > 180.f) ea2.z += (-1) * copysign(1.0, ea2.z) * 360.f;

	//while (ea1.x < 0.f && ea2.x < 0.f) { ea1.x += 360.f; ea2.x += 360.f; }
	//while (ea1.y < 0.f && ea2.y < 0.f) { ea1.y += 360.f; ea2.y += 360.f; }
	//while (ea1.z < 0.f && ea2.z < 0.f) { ea1.z += 360.f; ea2.z += 360.f; }

	glm::vec3 altEa1_1 = glm::vec3(ea1.x + 180.f, 180.f - ea1.y, ea1.z + 180.f);
	glm::vec3 altEa2_1 = glm::vec3(ea2.x + 180.f, 180.f - ea2.y, ea2.z + 180.f);

	std::vector<glm::vec3> ea1_vec = { ea1, altEa1_1 };
	std::vector<glm::vec3> ea2_vec = { ea2, altEa2_1 };

	for (int i = 0; i < 2; i++) {
		ea1_vec.push_back(ea1_vec[i] + glm::vec3(360.f, 0.f, 0.f));
		ea1_vec.push_back(ea1_vec[i] + glm::vec3(0.f, 360.f, 0.f));
		ea1_vec.push_back(ea1_vec[i] + glm::vec3(0.f, 0.f, 360.f));
		ea1_vec.push_back(ea1_vec[i] - glm::vec3(360.f, 0.f, 0.f));
		ea1_vec.push_back(ea1_vec[i] - glm::vec3(0.f, 360.f, 0.f));
		ea1_vec.push_back(ea1_vec[i] - glm::vec3(0.f, 0.f, 360.f));
		
		ea2_vec.push_back(ea2_vec[i] + glm::vec3(360.f, 0.f, 0.f));
		ea2_vec.push_back(ea2_vec[i] + glm::vec3(0.f, 360.f, 0.f));
		ea2_vec.push_back(ea2_vec[i] + glm::vec3(0.f, 0.f, 360.f));
		ea2_vec.push_back(ea2_vec[i] - glm::vec3(360.f, 0.f, 0.f));
		ea2_vec.push_back(ea2_vec[i] - glm::vec3(0.f, 360.f, 0.f));
		ea2_vec.push_back(ea2_vec[i] - glm::vec3(0.f, 0.f, 360.f));
	}

	std::vector<float> distances;
	for (int i = 0; i < ea1_vec.size(); i++) {
		for (int j = 0; j < ea2_vec.size(); j++) {
			distances.push_back(glm::distance(ea1_vec[i], ea2_vec[j]));
		}
	}

	int minIndex = std::min_element(distances.begin(), distances.end()) - distances.begin();
	int ea1Index = minIndex / ea2_vec.size();
	int ea2Index = minIndex % ea2_vec.size();

	ea1 = ea1_vec[ea1Index];
	ea2 = ea2_vec[ea2Index];
}

glm::vec3 normalizeEulerAngles(const glm::vec3& eulerAngles) {
	glm::vec3 normalized = eulerAngles;

	while (normalized.x > 180.f) normalized.x -= 360.f;
	while (normalized.x < -180.f) normalized.x += 360.f;
	while (normalized.y > 180.f) normalized.y -= 360.f;
	while (normalized.y < -180.f) normalized.y += 360.f;
	while (normalized.z > 180.f) normalized.z -= 360.f;
	while (normalized.z < -180.f) normalized.z += 360.f;

	return normalized;
}

struct RotationSet {
	glm::vec3 startEA;
	glm::vec3 endEA;
	glm::quat startQuat;
	glm::quat endQuat;

	RotationSet(glm::vec3 startEA, glm::vec3 endEA, glm::quat startQuat, glm::quat endQuat) :
		startEA(startEA), endEA(endEA), startQuat(startQuat), endQuat(endQuat) {
	};
};

struct SymParams {
	glm::vec3 startPos;
	glm::vec3 endPos;
	float speed;
	bool isEA;
	bool rightSlerp;
	glm::vec4 startParam;
	glm::vec4 endParam;
	bool animation;
	int frames;
	bool slerpTranslation;

	SymParams(glm::vec3 startPos, glm::vec3 endPos, float speed, bool isEA, bool rightSlerp, glm::vec4 startParam, glm::vec4 endParam, bool animation, int frames, bool slerpTranslation) :
		startPos(startPos), endPos(endPos), speed(speed), isEA(isEA), rightSlerp(rightSlerp), startParam(startParam), endParam(endParam), animation(animation), frames(frames), slerpTranslation(slerpTranslation) {}
};

struct SymData {
	std::vector<glm::mat4> leftModels;
	std::vector<glm::mat4> rightModels;
	float time;

	SymData() : time(0.f) {	}
};

struct SymMemory {
	SymParams params;
	SymData data;

	std::mutex mutex;
	std::atomic<bool> terminateThread;
	std::atomic<float> sleep_debt;

	SymMemory(glm::vec3 startPos, glm::vec3 endPos, float speed, bool isEA, bool rightSlerp, glm::vec4 startParam, glm::vec4 endParam, bool animation, int frames, bool slerpTranslation) :
		params(startPos, endPos, speed, isEA, rightSlerp, startParam, endParam, animation, frames, slerpTranslation), data()
	{
		terminateThread = false;
		sleep_debt = 0.f;
	}

	~SymMemory()
	{
		mutex.lock();
		terminateThread = true;
		mutex.unlock();
	}
};

RotationSet calcRotationSet(SymMemory* memory)
{
	glm::quat startQuat, endQuat;
	glm::vec3 startEA, endEA;

	if (memory->params.isEA) {
		startEA = glm::vec3(memory->params.startParam);
		endEA = glm::vec3(memory->params.endParam);

		startEA = normalizeEulerAngles(startEA);
		endEA = normalizeEulerAngles(endEA);
		normalizeTwoEulerAngles(startEA, endEA);

		startQuat = glm::quat(glm::radians(startEA));
		endQuat = glm::quat(glm::radians(endEA));
	}
	else {
		startQuat = glm::quat(memory->params.startParam);
		endQuat = glm::quat(memory->params.endParam);

		startEA = glm::degrees(glm::eulerAngles(startQuat));
		endEA = glm::degrees(glm::eulerAngles(endQuat));

		startEA = normalizeEulerAngles(startEA);
		endEA = normalizeEulerAngles(endEA);
		normalizeTwoEulerAngles(startEA, endEA);
	}

	return RotationSet(startEA, endEA, startQuat, endQuat);
};

std::pair<glm::mat4, glm::mat4> calculateModelMatrices(SymMemory* memory, RotationSet rotSet, float t)
{
	// position calculations
	glm::vec3 posLerp = glm::mix(
		memory->params.startPos,
		memory->params.endPos, t);
	glm::quat posSlerp = glm::slerp(
		glm::quat(0.f, memory->params.startPos.x, memory->params.startPos.y, memory->params.startPos.z),
		glm::quat(0.f, memory->params.endPos.x, memory->params.endPos.y, memory->params.endPos.z), t);
	glm::mat4 translateLerp = glm::translate(glm::mat4(1.f), posLerp);
	glm::mat4 translateSlerp = memory->params.slerpTranslation ? glm::translate(glm::mat4(1.f), glm::vec3(posSlerp.x, posSlerp.y, posSlerp.z)) : translateLerp;

	glm::vec3 angleLerp = glm::mix(rotSet.startEA, rotSet.endEA, t);
	glm::quat angleSlerp = memory->params.rightSlerp ? glm::slerp(rotSet.startQuat, rotSet.endQuat, t) : glm::lerp(rotSet.startQuat, rotSet.endQuat, t);
	glm::mat4 rotationLerp = createRotationMatrixFromEulerAngles(angleLerp);
	glm::mat4 rotationSlerp = glm::mat4_cast(angleSlerp);

	return { translateLerp * rotationLerp, translateSlerp * rotationSlerp };
};

void calculationThread(SymMemory* memory)
{
	std::chrono::high_resolution_clock::time_point calc_start, calc_end, wait_start;

	RotationSet rotSet = calcRotationSet(memory);

	while (!memory->terminateThread) {

		calc_start = std::chrono::high_resolution_clock::now();

		memory->mutex.lock();

		if (!memory->params.animation) {
			memory->data.leftModels = {};
			memory->data.rightModels = {};

			for (int i = 0; i < memory->params.frames; i++) {
				float t = i / (float)(memory->params.frames - 1);
				auto [modelLerp, modelSlerp] = calculateModelMatrices(memory, rotSet, t);
				memory->data.leftModels.push_back(modelLerp);
				memory->data.rightModels.push_back(modelSlerp);
			}

			memory->terminateThread = true;
		}
		else {
			memory->data.time += dt / 1000.f;

			// t calculations
			float fullDistance = glm::distance(memory->params.startPos, memory->params.endPos);
			float coveredDistance = memory->data.time * memory->params.speed;
			float t = fullDistance == 0 ? 0 : coveredDistance / fullDistance;
			if (t >= 1) {
				t = 1;
				memory->terminateThread = true;
			}

			auto [modelLerp, modelSlerp] = calculateModelMatrices(memory, rotSet, t);

			memory->data.leftModels = { modelLerp };
			memory->data.rightModels = { modelSlerp };
		}

		memory->mutex.unlock();

		calc_end = std::chrono::high_resolution_clock::now();

		float time2sleep = dt * 1000000.f - std::chrono::duration_cast<std::chrono::nanoseconds>(calc_end - calc_start).count() - memory->sleep_debt;

		wait_start = std::chrono::high_resolution_clock::now();

		if (time2sleep > 0) {
			while (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - wait_start).count() < time2sleep) {
				// busy waiting because std::this_thread::sleep_for sucks ass
			}
		}

		memory->sleep_debt = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - wait_start).count() - time2sleep;
	}
}