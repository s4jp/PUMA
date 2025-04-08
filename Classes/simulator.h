#pragma once

#include "glm/glm.hpp"

#include <atomic>
#include <mutex>
#include <glm/gtx/quaternion.hpp>
#include <vector>
#include <Frame.h>
#include <array>
#include <chrono>

static int dt = 10;		// in milliseconds
const Frame baseFrame = Frame();
const glm::mat4 F2initRot = glm::mat4_cast(glm::angleAxis((float)M_PI_2, glm::vec3(0.0f, 1.0f, 0.0f)));
const glm::mat4 F3initRot = glm::mat4_cast(glm::angleAxis((float)M_PI, glm::vec3(0.0f, 1.0f, 0.0f)));
const glm::mat4 F4initRot = F2initRot;

struct ConfigurationSpace {
	float alpha1;
	float alpha2;
	float q2;
	float alpha3;
	float alpha4;
	float alpha5;

	ConfigurationSpace(float alpha1, float alpha2, float q2, float alpha3, float alpha4, float alpha5) :
		alpha1(alpha1), alpha2(alpha2), q2(q2), alpha3(alpha3), alpha4(alpha4), alpha5(alpha5) {}

	ConfigurationSpace operator+(const ConfigurationSpace& other) {
		return ConfigurationSpace(
			alpha1 + other.alpha1,
			alpha2 + other.alpha2,
			q2 + other.q2,
			alpha3 + other.alpha3,
			alpha4 + other.alpha4,
			alpha5 + other.alpha5);
	}

	ConfigurationSpace operator*(const float& scalar) {
		return ConfigurationSpace(
			alpha1 * scalar,
			alpha2 * scalar,
			q2 * scalar,
			alpha3 * scalar,
			alpha4 * scalar,
			alpha5 * scalar);
	}

	void Print() {
		std::cout << "alpha1: " << alpha1 << std::endl;
		std::cout << "alpha2: " << alpha2 << std::endl;
		std::cout << "q2: " << q2 << std::endl;
		std::cout << "alpha3: " << alpha3 << std::endl;
		std::cout << "alpha4: " << alpha4 << std::endl;
		std::cout << "alpha5: " << alpha5 << std::endl;
		std::cout << std::endl;
	}
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
	std::array<Frame,5> frames;

	IKSet(Joints joints, ConfigurationSpace configSpace, std::array<Frame, 5> frames) :
		joints(joints), configSpace(configSpace), frames(frames) {}
};

struct SymParams {
	Frame startFrame;
	Frame endFrame;
	float speed;
	glm::vec3 lengths;

	SymParams(Frame startFrame, Frame endFrame, float speed, glm::vec3 lengths) :
		startFrame(startFrame), endFrame(endFrame), speed(speed), lengths(lengths) {}
};

struct SymData {
	std::array<glm::mat4, 5> leftModels;
	std::array<glm::mat4, 5> rightModels;
	std::array<float, 2> q2s;
	float time;
	glm::vec3 lengths;

	SymData() : time(0.f) {}
};

struct SymMemory {
	SymParams params;
	SymData data;

	std::mutex mutex;
	std::atomic<bool> terminateThread;
	std::atomic<float> sleep_debt;

	SymMemory(Frame startFrame, Frame endFrame, float speed, glm::vec3 lengths) :
		params(startFrame, endFrame, speed, lengths), data()
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

float normalizeAngle(const float angle) {
	float newAngle = angle;
	while (newAngle > M_PI) {
		newAngle -= 2 * M_PI;
	}
	while (newAngle < -M_PI) {
		newAngle += 2 * M_PI;
	}
	return newAngle;
}

bool isVec3NaN(const glm::vec3& vec) {
	return std::isnan(vec.x) || std::isnan(vec.y) || std::isnan(vec.z);
}

IKSet solveInverseKinematics(const Frame& effectorFrame, const glm::vec3& lengths, const IKSet* prevIKData)
{
	// calculate joints positions
	glm::vec3 p0 = baseFrame.GetOrigin();
	glm::vec3 p1 = p0;
	glm::vec3 p2 = p1 + baseFrame.GetZ() * lengths.x;

	glm::vec3 p5 = effectorFrame.GetOrigin();
	glm::vec3 p4 = p5 - effectorFrame.GetX() * lengths.z;

	glm::vec3 v40 = glm::normalize(p4 - p0);
	glm::vec3 v20 = glm::normalize(p2 - p0);
	glm::vec3 norm = glm::normalize(glm::cross(v40, v20));
	glm::vec3 v34n = glm::normalize(glm::cross(norm, effectorFrame.GetX()));

	glm::vec3 p3 = p4 + v34n * lengths.y;
	if (prevIKData != nullptr) {
		// set p3 to the closest point to the previous p3

		glm::vec3 p3alt = p4 - v34n * lengths.y;

		float distanceToPrev = glm::distance(p3, prevIKData->joints.p3);
		float altDistanceToPrev = glm::distance(p3alt, prevIKData->joints.p3);
		if (altDistanceToPrev < distanceToPrev)
			p3 = p3alt;
	}
	if (isVec3NaN(p3)) {
		// case when v34 and x5 are parallel

		if (prevIKData != nullptr) {
			float distance = glm::dot(prevIKData->joints.p3, norm);
			p3 = prevIKData->joints.p3 - norm * distance;
		}
		else {
			glm::vec3 v24 = glm::normalize(p2 - p4);
			p3 = p4 + v24 * lengths.y;
		}
	}
	if (isVec3NaN(norm)) {
		// case when v40 and v20 are parallel

		glm::vec3 v24 = glm::normalize(p2 - p4);
		p3 = p4 + v24 * lengths.y;
	}

	//-----------------------------------------------------------------------------//

	// calculate configuration space
	float q2 = glm::distance(p2, p3);

	// alpha1
	v40 = p4 - p0;
	float alpha1 = atan2(glm::dot(v40, baseFrame.GetY()), glm::dot(v40, baseFrame.GetX()));

	alpha1 = normalizeAngle(alpha1);
	Frame F1 = Frame(baseFrame);
	F1.Rotate(glm::angleAxis(alpha1, baseFrame.GetZ()));

	// alpha2
	glm::vec3 v32 = p3 - p2;
	float alpha2 = -atan2(glm::dot(v32, F1.GetZ()), glm::dot(v32, F1.GetX()));

	alpha2 = normalizeAngle(alpha2);
	Frame F2 = Frame(F1);
	F2.Translate(F1.GetZ() * lengths.x);
	F2.Rotate(glm::angleAxis(alpha2, F1.GetY()));

	// alpha3
	glm::vec3 v34 = p3 - p4;
	glm::vec3 x3 = glm::cross(F2.GetY(), glm::normalize(v34));
	float alpha3 = -atan2(glm::dot(x3, F2.GetZ()), glm::dot(x3, F2.GetX()));

	alpha3 = normalizeAngle(alpha3);
	Frame F3 = Frame(F2);
	F3.Translate(F2.GetX() * q2);
	F3.Rotate(glm::angleAxis(alpha3, F2.GetY()));

	// alpha4
	float alpha4 = atan2(glm::dot(effectorFrame.GetX(), F3.GetY()), glm::dot(effectorFrame.GetX(), F3.GetX()));

	alpha4 = normalizeAngle(alpha4);
	Frame F4 = Frame(F3);
	F4.Translate(F3.GetZ() * -lengths.y);
	F4.Rotate(glm::angleAxis(alpha4, F3.GetZ()));

	// alpha5
	glm::vec3 y4 = glm::cross(effectorFrame.GetX(), v34);
	float alpha5 = M_PI_2 - atan2(glm::dot(effectorFrame.GetZ(), v34), glm::dot(effectorFrame.GetZ(), y4));

	alpha5 = normalizeAngle(alpha5);
	Frame F5 = Frame(F4);
	F5.Translate(F4.GetX() * lengths.z);
	F5.Rotate(glm::angleAxis(alpha5, F4.GetX()));

	//// test found values
	//float p1dist = glm::distance(p1, F1.GetOrigin());
	//float p2dist = glm::distance(p2, F2.GetOrigin());
	//float p3dist = glm::distance(p3, F3.GetOrigin());
	//float p4dist = glm::distance(p4, F4.GetOrigin());
	//float p5dist = glm::distance(p5, F5.GetOrigin());
	//std::cout << "Cumulative distances: " << (p1dist + p2dist + p3dist + p4dist + p5dist) << std::endl;

	//glm::quat q5 = F5.GetRotation();
	//std::cout << "F5 rot dot effectorFrame: " << glm::abs(glm::dot(q5, effectorFrame.GetRotation())) << std::endl << std::endl;

	return IKSet(
		Joints(p1, p2, p3, p4, p5), 
		ConfigurationSpace(alpha1, alpha2, q2, alpha3, alpha4, alpha5), 
		{ F1, F2, F3, F4, F5 });
}

std::array<Frame, 5> calculateFramesFromConfSpace(ConfigurationSpace configSpace, glm::vec3 lengths)
{
	Frame F1 = Frame(baseFrame);
	F1.Rotate(glm::angleAxis(configSpace.alpha1, baseFrame.GetZ()));

	Frame F2 = Frame(F1);
	F2.Translate(F1.GetZ() * lengths.x);
	F2.Rotate(glm::angleAxis(configSpace.alpha2, F1.GetY()));

	Frame F3 = Frame(F2);
	F3.Translate(F2.GetX() * configSpace.q2);
	F3.Rotate(glm::angleAxis(configSpace.alpha3, F2.GetY()));

	Frame F4 = Frame(F3);
	F4.Translate(F3.GetZ() * -lengths.y);
	F4.Rotate(glm::angleAxis(configSpace.alpha4, F3.GetZ()));

	Frame F5 = Frame(F4);
	F5.Translate(F4.GetX() * lengths.z);
	F5.Rotate(glm::angleAxis(configSpace.alpha5, F4.GetX()));

	return { F1, F2, F3, F4, F5 };
}

Frame interpolateFrames(Frame startFrame, Frame endFrame, const float t)
{
	glm::vec3 posLerp = glm::mix(startFrame.GetOrigin(), endFrame.GetOrigin(), t);
	glm::quat angleSlerp = glm::slerp(startFrame.GetRotation(), endFrame.GetRotation(), t);
	return Frame(posLerp, angleSlerp);
}

ConfigurationSpace calculateIterpolationDirection(const ConfigurationSpace& startCS, const ConfigurationSpace& endCS)
{
	float angle1 = normalizeAngle(endCS.alpha1 - startCS.alpha1);
	float angle2 = normalizeAngle(endCS.alpha2 - startCS.alpha2);
	float q2 = endCS.q2 - startCS.q2;
	float angle3 = normalizeAngle(endCS.alpha3 - startCS.alpha3);
	float angle4 = normalizeAngle(endCS.alpha4 - startCS.alpha4);
	float angle5 = normalizeAngle(endCS.alpha5 - startCS.alpha5);

	return ConfigurationSpace(angle1, angle2, q2, angle3, angle4, angle5);
}

void calculationThread(SymMemory* memory)
{
	std::chrono::high_resolution_clock::time_point calc_start, calc_end, wait_start;

	IKSet startIK = solveInverseKinematics(memory->params.startFrame, memory->params.lengths, nullptr);
	IKSet endIK = solveInverseKinematics(memory->params.endFrame, memory->params.lengths, nullptr);
	ConfigurationSpace directions = calculateIterpolationDirection(startIK.configSpace, endIK.configSpace);

	IKSet prevIK = startIK;

	memory->data.lengths = memory->params.lengths;

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
		ConfigurationSpace currCS = startIK.configSpace + directions * t;
		std::array<Frame, 5> currFrames = calculateFramesFromConfSpace(currCS, memory->params.lengths);
		IKSet currIK = solveInverseKinematics(interpolateFrames(memory->params.startFrame, memory->params.endFrame, t), memory->params.lengths, &prevIK);
		prevIK = currIK;

		// F1, F2, F3, F4, F5
		memory->data.leftModels = {
			currFrames.at(0).GetMatrix(),
			currFrames.at(1).GetMatrix() * F2initRot,
			currFrames.at(2).GetMatrix() * F3initRot,
			currFrames.at(3).GetMatrix() * F4initRot,
			currFrames.at(4).GetMatrix()
		};
		memory->data.rightModels = {
			currIK.frames.at(0).GetMatrix(),
			currIK.frames.at(1).GetMatrix() * F2initRot,
			currIK.frames.at(2).GetMatrix() * F3initRot,
			currIK.frames.at(3).GetMatrix() * F4initRot,
			currIK.frames.at(4).GetMatrix()
		};

		memory->data.q2s = { 
			currCS.q2,
			currIK.configSpace.q2 
		};

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