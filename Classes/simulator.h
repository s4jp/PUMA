#pragma once

#include "glm/glm.hpp"

#include <atomic>
#include <mutex>
#include <glm/gtx/quaternion.hpp>
#include <vector>
#include <Frame.h>
#include <array>

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
	std::array<Frame,5> frames;

	IKSet(Joints joints, ConfigurationSpace configSpace, std::array<Frame, 5> frames) :
		joints(joints), configSpace(configSpace), frames(frames) {}
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
	std::array<glm::mat4, 5> leftModels;
	std::array<glm::mat4, 5> rightModels;
	std::array<float, 2> q2s;
	float time;

	SymData() : time(0.f) {}
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

	//-----------------------------------------------------------------------------//

	// calculate configuration space
	float q2 = glm::distance(p2, p3);

	// todo: handle v30 vs v40
	v40 = p4 - p0;
	float alpha1 = atan2(glm::dot(v40, baseFrame.GetY()), glm::dot(v40, baseFrame.GetX()));
	Frame F1 = Frame(baseFrame);
	F1.Rotate(glm::angleAxis(alpha1, baseFrame.GetZ()));

	glm::vec3 v32 = p3 - p2;
	float alpha2 = -atan2(glm::dot(v32, F1.GetZ()), glm::dot(v32, F1.GetX()));
	Frame F2 = Frame(F1);
	F2.Translate(F1.GetZ() * l1);
	F2.Rotate(glm::angleAxis(alpha2, F1.GetY()));

	glm::vec3 x3 = glm::normalize(glm::cross(F2.GetY(), glm::normalize(p3 - p4)));
	float alpha3 = -atan2(glm::dot(x3, F2.GetZ()), glm::dot(x3, F2.GetX()));
	Frame F3 = Frame(F2);
	F3.Translate(F2.GetX() * q2);
	F3.Rotate(glm::angleAxis(alpha3, F2.GetY()));

	glm::vec3 x5 = effectorFrame.GetX();
	float alpha4 = atan2(glm::dot(x5, F3.GetY()), glm::dot(x5, F3.GetX()));
	Frame F4 = Frame(F3);
	F4.Translate(F3.GetZ() * -l3);
	F4.Rotate(glm::angleAxis(alpha4, F3.GetZ()));

	glm::vec3 z4 = p3 - p4;
	glm::vec3 y4 = glm::cross(z4, x5);
	glm::vec3 z5 = effectorFrame.GetZ();
	float alpha5 = M_PI / 2.0f - atan2(glm::dot(z5, z4), -glm::dot(z5, y4));
	Frame F5 = Frame(F4);
	F5.Translate(F4.GetX() * l4);
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

Frame interpolateFrames(Frame startFrame, Frame endFrame, const float t)
{
	glm::vec3 posLerp = glm::mix(startFrame.GetOrigin(), endFrame.GetOrigin(), t);
	glm::quat angleSlerp = glm::slerp(startFrame.GetRotation(), endFrame.GetRotation(), t);
	return Frame(posLerp, angleSlerp);
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

		Frame interpolatedFrame = interpolateFrames(memory->params.startFrame, memory->params.endFrame, t);
		//std::cout << "t: " << t << std::endl;
		//CAD::printVector(interpolatedFrame.GetOrigin());
		//std::cout << std::endl;
		
		IKSet currIK = solveInverseKinematics(interpolatedFrame,
			memory->params.l1, memory->params.l3, memory->params.l4);

		// DEBUG PRINT
		if (t == 1) {
			//compare GetModel() with GetMatrix()
			glm::mat4 model1 = currIK.frames.at(0).GetModel();
			glm::mat4 model2 = currIK.frames.at(1).GetModel();
			glm::mat4 model3 = currIK.frames.at(2).GetModel();
			glm::mat4 model4 = currIK.frames.at(3).GetModel();
			glm::mat4 model5 = currIK.frames.at(4).GetModel();

			glm::mat4 matrix1 = currIK.frames.at(0).GetMatrix();
			glm::mat4 matrix2 = currIK.frames.at(1).GetMatrix();
			glm::mat4 matrix3 = currIK.frames.at(2).GetMatrix();
			glm::mat4 matrix4 = currIK.frames.at(3).GetMatrix();
			glm::mat4 matrix5 = currIK.frames.at(4).GetMatrix();

			std::cout << "M1: " << std::endl;
			CAD::printMatrix(model1);
			std::cout << "----------------------------" << std::endl;
			CAD::printMatrix(matrix1);
			std::cout << std::endl;

			std::cout << "M2: " << std::endl;
			CAD::printMatrix(model2);
			std::cout << "----------------------------" << std::endl;
			CAD::printMatrix(matrix2);
			std::cout << std::endl;

			std::cout << "M3: " << std::endl;
			CAD::printMatrix(model3);
			std::cout << "----------------------------" << std::endl;
			CAD::printMatrix(matrix3);
			std::cout << std::endl;

			std::cout << "M4: " << std::endl;
			CAD::printMatrix(model4);
			std::cout << "----------------------------" << std::endl;
			CAD::printMatrix(matrix4);
			std::cout << std::endl;

			std::cout << "M5: " << std::endl;
			CAD::printMatrix(model5);
			std::cout << "----------------------------" << std::endl;
			CAD::printMatrix(matrix5);
			std::cout << std::endl;
		}

		memory->data.leftModels = {
			currIK.frames.at(0).GetModel(),
			currIK.frames.at(1).GetModel(),
			currIK.frames.at(2).GetModel(),
			currIK.frames.at(3).GetModel(),
			currIK.frames.at(4).GetModel()
		};

		memory->data.rightModels = {
			currIK.frames.at(0).GetMatrix(),
			currIK.frames.at(1).GetMatrix(),
			currIK.frames.at(2).GetMatrix(),
			currIK.frames.at(3).GetMatrix(),
			currIK.frames.at(4).GetMatrix()
		};

		memory->data.q2s = { currIK.configSpace.q2, currIK.configSpace.q2 };

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