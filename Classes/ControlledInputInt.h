#pragma once

#include <imgui.h>
#include <imgui_stdlib.h>

class ControlledInputInt {
public:
	ControlledInputInt(std::string label, int value, int step = 1, int lowerBound = -INT32_MAX, int upperBound = INT32_MAX) {
		this->label = label;
		this->value = value;
		this->step = step;
		this->lowerBound = lowerBound;
		this->upperBound = upperBound;
	}

	bool Render() {
		bool change = false;
		if (ImGui::InputInt(label.c_str(), &value, step, step * 10))
		{
			change = true;
			if (value < lowerBound) {
				value = lowerBound;
			}
			else if (value > upperBound) {
				value = upperBound;
			}
		}
		return change;
	}

	float GetValue() const {
		return value;
	}

	int* GetPointer() {
		return &value;
	}

private:
	std::string label;
	int value;
	int step;
	int lowerBound;
	int upperBound;
};