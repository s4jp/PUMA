#pragma once

#include "glm/glm.hpp"

#include <atomic>
#include <mutex>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>

static int dt = 10;		// in milliseconds

struct SymParams {
	glm::vec3 startPos;
	glm::vec3 endPos;
	float speed;
	glm::quat startQuat;
	glm::quat endQuat;

	SymParams(glm::vec3 startPos, glm::vec3 endPos, float speed, glm::quat startQuat, glm::quat endQuat) :
		startPos(startPos), endPos(endPos), speed(speed), startQuat(startQuat), endQuat(endQuat) {}
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

	SymMemory(glm::vec3 startPos, glm::vec3 endPos, float speed, glm::quat startQuat, glm::quat endQuat) :
		params(startPos, endPos, speed, startQuat, endQuat), data()
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

std::pair<glm::mat4, glm::mat4> calculateModelMatrices(SymMemory* memory, float t)
{
	glm::vec3 posLerp = glm::mix(
		memory->params.startPos,
		memory->params.endPos, t);
	glm::mat4 translateLerp = glm::translate(glm::mat4(1.f), posLerp);

	glm::quat angleSlerp = glm::slerp(memory->params.startQuat, memory->params.endQuat, t);
	glm::mat4 rotationSlerp = glm::mat4_cast(angleSlerp);

	// temporary return same values
	return { translateLerp * rotationSlerp, translateLerp * rotationSlerp };
};

void calculationThread(SymMemory* memory)
{
	std::chrono::high_resolution_clock::time_point calc_start, calc_end, wait_start;

	while (!memory->terminateThread) {

		calc_start = std::chrono::high_resolution_clock::now();

		memory->mutex.lock();

		memory->data.time += dt / 1000.f;

		// t calculations
		float fullDistance = glm::distance(memory->params.startPos, memory->params.endPos);
		float coveredDistance = memory->data.time * memory->params.speed;
		float t = fullDistance == 0 ? 0 : coveredDistance / fullDistance;
		if (t >= 1) {
			t = 1;
			memory->terminateThread = true;
		}

		auto [modelLeft, modelRight] = calculateModelMatrices(memory, t);

		memory->data.leftModels = { modelLeft };
		memory->data.rightModels = { modelRight };

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