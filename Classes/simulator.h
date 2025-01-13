#pragma once

#include "glm/glm.hpp"

#include <atomic>
#include <mutex>
#include <glm/gtx/quaternion.hpp>
#include <vector>
#include <Frame.h>

static int dt = 10;		// in milliseconds
const Frame baseFrame = Frame();

struct ConfigurationSpace {
	float alpha1;
	float alpha2;
	float q2;
	float alpha3;
	float alpha4;
	float alpha5;

	ConfigurationSpace(float alpha1, float alpha2, float q2, float alpha3, float alpha4, float alpha5) :
		alpha1(alpha1), alpha2(alpha2), q2(q2), alpha3(alpha3), alpha4(alpha4), alpha5(alpha5) {}
};

struct Joints {
	glm::vec3 p1;
	glm::vec3 p2;
	glm::vec3 p3;
	glm::vec3 p4;
	glm::vec3 p5;

	Joints(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4, glm::vec3 p5) :
		p1(p1), p2(p2), p3(p3), p4(p4), p5(p5) {}
};

struct IKSet {
	Joints joints;
	ConfigurationSpace configSpace;

	IKSet(Joints joints, ConfigurationSpace configSpace) :
		joints(joints), configSpace(configSpace) {}
};

struct SymParams {
	Frame startFrame;
	Frame endFrame;
	float speed;
	float l1;
	float l3;
	float l4;

	SymParams(Frame startFrame, Frame endFrame, float speed, float l1, float l3, float l4) :
		startFrame(startFrame), endFrame(endFrame), speed(speed), l1(l1), l3(l3), l4(l4) {}
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

	SymMemory(Frame startFrame, Frame endFrame, float speed, float l1, float l3, float l4) :
		params(startFrame, endFrame, speed, l1, l3, l4), data()
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

//std::pair<glm::mat4, glm::mat4> calculateModelMatrices(SymMemory* memory, float t)
//{
//	glm::vec3 posLerp = glm::mix(
//		memory->params.startPos,
//		memory->params.endPos, t);
//	glm::mat4 translateLerp = glm::translate(glm::mat4(1.f), posLerp);
//
//	glm::quat angleSlerp = glm::slerp(memory->params.startQuat, memory->params.endQuat, t);
//	glm::mat4 rotationSlerp = glm::mat4_cast(angleSlerp);
//
//	// temporary return same values
//	return { translateLerp * rotationSlerp, translateLerp * rotationSlerp };
//};

IKSet solveInverseKinematics(const Frame& effectorFrame, const float l1, const float l3, const float l4) 
{
	// calculate joints positions
	glm::vec3 p0 = baseFrame.GetOrigin();
	glm::vec3 p1 = p0;
	glm::vec3 p2 = p1 + baseFrame.GetZ() * l1;

	glm::vec3 p5 = effectorFrame.GetOrigin();
	glm::vec3 p4 = p5 - effectorFrame.GetX() * l4;

	glm::vec3 v40 = glm::normalize(p4 - p0);
	glm::vec3 v20 = glm::normalize(p2 - p0);
	glm::vec3 norm = glm::normalize(glm::cross(v40, v20));
	// todo: check if v40 and v20 are parallel

	glm::vec3 v34 = glm::normalize(glm::cross(norm, effectorFrame.GetX()));
	// todo: check if v34 and efectorFrame.GetX() are parallel
	glm::vec3 p3 = p4 - v34 * l3;
	// todo: handle +- v34

	Joints joints(p1, p2, p3, p4, p5);

	//-----------------------------------------------------------------------------//

	// calculate configuration space
	float q2 = glm::distance(p2, p3);

	// todo: handle v30 vs v40
	v40 = p4 - p0;
	float alpha1 = atan2(glm::dot(v40, baseFrame.GetY()), glm::dot(v40, baseFrame.GetX()));
	Frame F1 = baseFrame;
	F1.Rotate(glm::angleAxis(alpha1, baseFrame.GetZ()));

	glm::vec3 v32 = p3 - p2;
	float alpha2 = -atan2(glm::dot(v32, F1.GetZ()), glm::dot(v32, F1.GetX()));
	Frame F2 = F1;
	F2.Translate(F1.GetZ() * l1);
	F2.Rotate(glm::angleAxis(alpha2, F1.GetY()));

	glm::vec3 x3 = glm::normalize(glm::cross(F2.GetY(), glm::normalize(p3 - p4)));
	float alpha3 = -atan2(glm::dot(x3, F2.GetZ()), glm::dot(x3, F2.GetX()));
	Frame F3 = F2;
	F3.Translate(F2.GetX() * q2);
	F3.Rotate(glm::angleAxis(alpha3, F2.GetY()));

	glm::vec3 x5 = effectorFrame.GetX();
	float alpha4 = atan2(glm::dot(x5, F3.GetY()), glm::dot(x5, F3.GetX()));
	Frame F4 = F3;
	F4.Translate(F3.GetZ() * -l3);
	F4.Rotate(glm::angleAxis(alpha4, F3.GetZ()));

	glm::vec3 z4 = p3 - p4;
	glm::vec3 y4 = glm::cross(z4, x5);
	glm::vec3 z5 = effectorFrame.GetZ();
	float alpha5 = M_PI / 2.0f - atan2(glm::dot(z5, z4), -glm::dot(z5, y4));
	Frame F5 = F4;
	F5.Translate(F4.GetX() * l4);
	F5.Rotate(glm::angleAxis(alpha5, F4.GetX()));

	ConfigurationSpace configSpace(alpha1, alpha2, q2, alpha3, alpha4, alpha5);

	return IKSet(joints, configSpace);
}

void calculationThread(SymMemory* memory)
{
	std::chrono::high_resolution_clock::time_point calc_start, calc_end, wait_start;

	IKSet startIK = solveInverseKinematics(memory->params.startFrame, memory->params.l1, memory->params.l3, memory->params.l4);
	IKSet endIK = solveInverseKinematics(memory->params.endFrame, memory->params.l1, memory->params.l3, memory->params.l4);

	while (!memory->terminateThread) {

		calc_start = std::chrono::high_resolution_clock::now();

		memory->mutex.lock();

		memory->data.time += dt / 1000.f;

		// t calculations
		float t = memory->data.time * memory->params.speed / 100.f;
		if (t >= 1) {
			t = 1;
			memory->terminateThread = true;
		}

		//auto [modelLeft, modelRight] = calculateModelMatrices(memory, t);

		//memory->data.leftModels = { modelLeft };
		//memory->data.rightModels = { modelRight };

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