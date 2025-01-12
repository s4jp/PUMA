#define _USE_MATH_DEFINES
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_stdlib.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>
#include <vector>
#include <string>

#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"

#include "Shader.h"
#include "VAO.h"
#include "VBO.h"
#include "EBO.h"
#include "Camera.h"
#include "Grid.h"
#include "pointer.h"
#include "ControlledInputFloat.h"
#include "ControlledInputInt.h"
#include "simulator.h"
#include <thread>

const float near = 0.1f;
const float far = 300.0f;
const int dispCount = 2;

Camera *camera;
Grid* grid;
std::vector<Pointer*> frame;

glm::mat4 view;
glm::mat4 proj;

void window_size_callback(GLFWwindow *window, int width, int height);

int viewLoc, projLoc, colorLoc;
int phongModelLoc, phongViewLoc, phongProjLoc, phongColorLoc;

ControlledInputFloat speed("Speed", 2.f, 0.01f, 0.01f);
glm::vec3 startPos(0.f);
glm::vec3 endPos(3.f, 6.f, 9.f);
static int mode = 0;
glm::vec3 startEA(0.f);
glm::vec3 endEA(0.f);
glm::quat startQ(1.f, 0.f, 0.f, 0.f);
glm::quat endQ(1.f, 0.f, 0.f, 0.f);

SymMemory* memory;
SymData data;
std::thread calcThread;

int main() { 
    // initial values
    int width = 1800;
    int height = 800;
    glm::vec3 cameraPosition = glm::vec3(25.0f, 25.0f, 25.0f);
    float fov = M_PI / 4.0f;
    int guiWidth = 300;

    #pragma region gl_boilerplate
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(width, height, "PUMA", NULL, NULL);
    if (window == NULL) {
      std::cout << "Failed to create GLFW window" << std::endl;
      glfwTerminate();
      return -1;
    }
    glfwMakeContextCurrent(window);

    gladLoadGL();
    glEnable(GL_DEPTH_TEST);

    GLFWimage icon;
    icon.pixels = stbi_load("icon.png", &icon.width, &icon.height, 0, 4);
    glfwSetWindowIcon(window, 1, &icon);
    stbi_image_free(icon.pixels);
    #pragma endregion

    // shaders and uniforms
    Shader shaderProgram("Shaders\\default.vert", "Shaders\\default.frag");
    viewLoc = glGetUniformLocation(shaderProgram.ID, "view");
    projLoc = glGetUniformLocation(shaderProgram.ID, "proj");
    colorLoc = glGetUniformLocation(shaderProgram.ID, "color");

    Shader phongShader("Shaders\\phong.vert", "Shaders\\phong.frag");
	phongModelLoc = glGetUniformLocation(phongShader.ID, "model");
    phongViewLoc = glGetUniformLocation(phongShader.ID, "view");
    phongProjLoc = glGetUniformLocation(phongShader.ID, "proj");
    phongColorLoc = glGetUniformLocation(phongShader.ID, "objectColor");

    // callbacks
    glfwSetWindowSizeCallback(window, window_size_callback);

    camera = new Camera(width, height, cameraPosition, fov, near, far, guiWidth, dispCount);
    camera->PrepareMatrices(view, proj);

    grid = new Grid();
	frame.push_back(new Pointer("Meshes\\pointerX.obj", glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)));
	frame.push_back(new Pointer("Meshes\\pointerY.obj", glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)));
	frame.push_back(new Pointer("Meshes\\pointerZ.obj", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)));

    #pragma region imgui_boilerplate
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 460");
    #pragma endregion

	// simulation
    memory = new SymMemory(startPos, endPos, speed.GetValue(), mode == 0, glm::vec4(), glm::vec4());
    data = memory->data;
    calcThread = std::thread(calculationThread, memory);

    while (!glfwWindowShouldClose(window)) 
    {
        #pragma region init
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::SetNextWindowSize(ImVec2(camera->guiWidth, camera->GetHeight()));
        ImGui::SetNextWindowPos(ImVec2(camera->GetWidth() * dispCount, 0));
        #pragma endregion

        camera->HandleInputs(window);
        camera->PrepareMatrices(view, proj);

		memory->mutex.lock();
		data = memory->data;
		memory->mutex.unlock();
        
        // render non-grayscaleable objects
        shaderProgram.Activate();

        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(proj));

        // render left side
        glViewport(0, 0, camera->GetWidth(), camera->GetHeight());
        grid->Render(colorLoc);

        // render right side
        glViewport(camera->GetWidth(), 0, camera->GetWidth(), camera->GetHeight());
        grid->Render(colorLoc);

		// render shaded objects
		phongShader.Activate();

		glUniformMatrix4fv(phongViewLoc, 1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(phongProjLoc, 1, GL_FALSE, glm::value_ptr(proj));

		// render left side
        glViewport(0, 0, camera->GetWidth(), camera->GetHeight());
		for (auto& model : data.leftModels) {
			glUniformMatrix4fv(phongModelLoc, 1, GL_FALSE, glm::value_ptr(model));
			for (auto& pointer : frame) {
				pointer->Render(phongColorLoc);
			}
		}

		// render right side
        glViewport(camera->GetWidth(), 0, camera->GetWidth(), camera->GetHeight());
		for (auto& model : data.rightModels) {
			glUniformMatrix4fv(phongModelLoc, 1, GL_FALSE, glm::value_ptr(model));
			for (auto& pointer : frame) {
				pointer->Render(phongColorLoc);
			}
		}

        // imgui rendering
        ImGui::Begin("Menu", 0,
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

        ImGui::SeparatorText("Position:");
		ImGui::InputFloat("X_start",    &startPos.x,   0.01f, 0.1f, "%.2f");
		ImGui::InputFloat("Y_start",    &startPos.y,   0.01f, 0.1f, "%.2f");
		ImGui::InputFloat("Z_start",    &startPos.z,   0.01f, 0.1f, "%.2f");
        ImGui::Spacing();
		ImGui::InputFloat("X_end",      &endPos.x,     0.01f, 0.1f, "%.2f");
		ImGui::InputFloat("Y_end",      &endPos.y,     0.01f, 0.1f, "%.2f");
		ImGui::InputFloat("Z_end",      &endPos.z,     0.01f, 0.1f, "%.2f");

        ImGui::SeparatorText("Rotation:");
		ImGui::RadioButton("Euler angles", &mode, 0); ImGui::SameLine();
		ImGui::RadioButton("Quaternions", &mode, 1);
        ImGui::Spacing();

        switch (mode) {
		case 0:
			ImGui::Text("Unit - degrees");
            ImGui::Spacing();
			ImGui::InputFloat("pitch_start",    &startEA.x, 0.1f, 1.f, "%.1f");
			ImGui::InputFloat("yaw_start",      &startEA.y, 0.1f, 1.f, "%.1f");
			ImGui::InputFloat("roll_start",     &startEA.z, 0.1f, 1.f, "%.1f");
            ImGui::Spacing();
			ImGui::InputFloat("pitch_end",      &endEA.x,   0.1f, 1.f, "%.1f");
			ImGui::InputFloat("yaw_end",        &endEA.y,   0.1f, 1.f, "%.1f");
			ImGui::InputFloat("roll_end",       &endEA.z,   0.1f, 1.f, "%.1f");
			break;
		case 1:
			ImGui::InputFloat("x_start",        &startQ.x,  0.01f, 0.1f, "%.2f");
			ImGui::InputFloat("y_start",        &startQ.y,  0.01f, 0.1f, "%.2f");
			ImGui::InputFloat("z_start",        &startQ.z,  0.01f, 0.1f, "%.2f");
			ImGui::InputFloat("scalar_start",   &startQ.w,  0.01f, 0.1f, "%.2f");
            ImGui::Spacing();
			ImGui::InputFloat("x_end",          &endQ.x,    0.01f, 0.1f, "%.2f");
			ImGui::InputFloat("y_end",          &endQ.y,    0.01f, 0.1f, "%.2f");
			ImGui::InputFloat("z_end",          &endQ.z,    0.01f, 0.1f, "%.2f");
			ImGui::InputFloat("scalar_end",     &endQ.w,    0.01f, 0.1f, "%.2f");
			break;
        }
        ImGui::Spacing();
        ImGui::SeparatorText("Options:");
        speed.Render();

        ImGui::Spacing();
        if (ImGui::Button("Run", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
			memory->mutex.lock();
			memory->terminateThread = true;
			memory->mutex.unlock();
			calcThread.join();

            if (mode == 0) {
                memory = new SymMemory(startPos, endPos, speed.GetValue(), true, glm::vec4(startEA, 0), glm::vec4(endEA, 0));
			}
            else {
                memory = new SymMemory(startPos, endPos, speed.GetValue(), false, glm::vec4(startQ.x, startQ.y, startQ.z, startQ.w), glm::vec4(endQ.x, endQ.y, endQ.z, endQ.w));
            }
            data = memory->data;
			calcThread = std::thread(calculationThread, memory);
        }

        ImGui::End();
        #pragma region rest
        ImGui::Render();
        //std::cout << ImGui::GetIO().Framerate << std::endl;
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
        #pragma endregion
    }
    #pragma region exit
	memory->terminateThread = true;
    calcThread.join();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    shaderProgram.Delete();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
    #pragma endregion
}

// callbacks
void window_size_callback(GLFWwindow *window, int width, int height) {
  camera->SetWidth(width);
  camera->SetHeight(height);
  camera->PrepareMatrices(view, proj);
}