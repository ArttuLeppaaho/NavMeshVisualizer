// Author: Arttu Leppäaho arttu.leppaaho@hotmail.com
// Date: 4.2022

#include "window.h"
#include "navmesh.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <string>
#include <chrono>
#include <algorithm>

// GLSL vertex shader source code for a simple vertex shader
const std::string VERTEX_SHADER_SOURCE =
"#version 330 core\n"
"layout(location = 0) in vec3 vertexPosition;\n"
"uniform mat4 u_modelViewProjection;\n"
"void main()\n"
"{\n"
"	gl_Position = u_modelViewProjection * vec4(vertexPosition, 1.0);\n"
"}\n";

// GLSL fragment shader source code for a simple fragment shader
const std::string FRAGMENT_SHADER_SOURCE =
"#version 330 core\n"
"uniform vec4 u_color;"
"uniform float u_depthOffset;"
"out vec4 fragColor;"
"void main()\n"
"{\n"
"	fragColor = u_color;\n"
"	gl_FragDepth = gl_FragCoord.z + u_depthOffset;\n"
"}\n";

const uint16_t DEFAULT_WINDOW_WIDTH = 800;
const uint16_t DEFAULT_WINDOW_HEIGHT = 600;

const std::string DEFAULT_MESH_PATH = "example_navmesh.obj";

const size_t MAX_SELECTED_TRIANGLES = 2;

uint32_t Window::shaderProgram_ = 0;

Window::Window() : valid_(true)
{
	selectedTriangles_[0] = -1;
	selectedTriangles_[1] = -1;

	glfwInit();

	// Create GLFW window
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_VISIBLE, GL_FALSE);

	glfwWindow_ = glfwCreateWindow(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT, "Navigation Mesh Visualizer", NULL, NULL);

	if (glfwWindow_ == NULL)
	{
		std::cerr << "Failed to create GLFW window!" << std::endl;

		valid_ = false;

		return;
	}

	glfwMakeContextCurrent(glfwWindow_);

	// Connect GLFW window callbacks
	glfwSetKeyCallback(glfwWindow_, Window::KeyCallback);
	glfwSetMouseButtonCallback(glfwWindow_, Window::MouseButtonCallback);
	glfwSetCursorPosCallback(glfwWindow_, Window::CursorPosCallback);
	glfwSetWindowSizeCallback(glfwWindow_, Window::WindowSizeCallback);
	glfwSetInputMode(glfwWindow_, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
	glfwSetWindowUserPointer(glfwWindow_, this);

	// Initialize GLAD
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cerr << "Failed to initialize GLAD!" << std::endl;

		valid_ = false;

		return;
	}

	// Set miscellaneous OpenGL parameters for a better looking view
	glClearColor(0.4f, 0.4f, 0.4f, 1.0f);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);

	// Compile shaders
	const int vertexShaderSourceSize = static_cast<int>(VERTEX_SHADER_SOURCE.size());
	const char* vertexShaderSourceData = VERTEX_SHADER_SOURCE.data();

	uint32_t vertexShader;
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexShaderSourceData, &vertexShaderSourceSize);
	glCompileShader(vertexShader);

	int success;
	char infoLog[512];
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);

	if (!success)
	{
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		std::cerr << "Vertex shader compilation failed!\n" << infoLog << std::endl;
	}

	const int fragmentShaderSourceSize = static_cast<int>(FRAGMENT_SHADER_SOURCE.size());
	const GLchar* fragmentShaderSourceData = FRAGMENT_SHADER_SOURCE.data();

	uint32_t fragmentShader;
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentShaderSourceData, &fragmentShaderSourceSize);
	glCompileShader(fragmentShader);

	if (!success)
	{
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		std::cerr << "Fragment shader compilation failed!\n" << infoLog << std::endl;
	}

	if (shaderProgram_ != 0)
	{
		valid_ = false;

		return;
	}

	shaderProgram_ = glCreateProgram();

	glAttachShader(shaderProgram_, vertexShader);
	glAttachShader(shaderProgram_, fragmentShader);
	glLinkProgram(shaderProgram_);

	glGetProgramiv(shaderProgram_, GL_LINK_STATUS, &success);

	if (!success)
	{
		glGetProgramInfoLog(shaderProgram_, 512, NULL, infoLog);
		std::cerr << "Shader program compilation failed!\n" << infoLog << std::endl;
	}

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

	glUseProgram(shaderProgram_);

	perspectiveProjection_ = GetPerspectiveProjection(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);

	// Create navigation mesh from file
	std::string meshFilePath;

	std::cout << "Enter the file path of the .obj file you would like to use as the navigation mesh:" << std::endl;
	std::cout << "Leave empty to use \"example_navmesh.obj\"." << std::endl;

	while (navMesh_ == nullptr)
	{
		std::getline(std::cin, meshFilePath);

		if (meshFilePath == "")
		{
			meshFilePath = DEFAULT_MESH_PATH;
		}

		navMesh_ = new NavMesh(meshFilePath);

		if (!navMesh_->IsValid())
		{
			delete navMesh_;
			navMesh_ = nullptr;

			std::cout << "Given file failed to load. Try again:" << std::endl;
		}
	}

	std::cout << ".obj file loaded successfully." << std::endl << std::endl;
}

Window::~Window()
{
	delete navMesh_;

	glUseProgram(0);
	glDeleteProgram(shaderProgram_);

	glfwTerminate();
}

uint32_t Window::GetShaderProgram()
{
	return shaderProgram_;
}

void Window::Run()
{
	if (!valid_)
	{
		return;
	}

	glfwShowWindow(glfwWindow_);

	std::cout << "MOVEMENT" << std::endl;
	std::cout << "--------" << std::endl;
	std::cout << "Move the camera with the WASD keys, left control and spacebar." << std::endl;
	std::cout << "Rotate the camera by dragging with the right mouse button." << std::endl;
	std::cout << "Move faster by holding down left shift while moving." << std::endl;
	std::cout << std::endl << "PATHFINDING" << std::endl;
	std::cout << "-----------" << std::endl;
	std::cout << "The program calculates and visualizes paths between selected triangles." << std::endl;
	std::cout << "Select triangles by left clicking them." << std::endl;
	std::cout << "Press Q to swap between selecting the start and end triangle." << std::endl;
	std::cout << "Paths are displayed in blue, other processed triangles are displayed in orange." << std::endl;
	std::cout << "The start of the path is displayed in green and the end is displayed in red." << std::endl;
	std::cout << std::endl << "OTHER" << std::endl;
	std::cout << "-----" << std::endl;
	std::cout << "Quit with ESC or by simply closing either of the windows." << std::endl;

	float cameraYaw = 0.0f;
	float cameraPitch = 0.0f;

	auto startTime = std::chrono::high_resolution_clock::now();
	float previousTime(0.0f);

	// Main loop
	while (!glfwWindowShouldClose(glfwWindow_))
	{
		float time = std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::high_resolution_clock::now() - startTime).count();
		float deltaTime = time - previousTime;

		previousTime = time;

		// Move camera
		glm::vec3 cameraVelocity(0.0);

		glm::vec2 deltaCursor = rightMousePressed_ ? cursorPosition_ - previousCursorPosition_ : glm::vec2(0.0f);

		if (forwardPressed_)
		{
			cameraVelocity.z -= 1.0f;
		}

		if (backwardPressed_)
		{
			cameraVelocity.z += 1.0f;
		}

		if (leftPressed_)
		{
			cameraVelocity.x -= 1.0f;
		}

		if (rightPressed_)
		{
			cameraVelocity.x += 1.0f;
		}

		if (upwardPressed_)
		{
			cameraVelocity.y += 1.0f;
		}

		if (downwardPressed_)
		{
			cameraVelocity.y -= 1.0f;
		}

		if (cameraVelocity != glm::vec3(0.0f))
		{
			cameraVelocity = glm::normalize(cameraVelocity);
		}

		if (shiftPressed_)
		{
			cameraVelocity *= 5.0f;
		}

		// Rotate camera
		cameraPitch = std::min(std::max(cameraPitch - deltaCursor.y * 0.01f, -glm::half_pi<float>()), glm::half_pi<float>());
		cameraYaw = cameraYaw - deltaCursor.x * 0.01f;

		// Apply camera position and rotation
		cameraRotation_ = glm::angleAxis(cameraPitch, glm::vec3(-1.0f, 0.0f, 0.0f)) * glm::angleAxis(cameraYaw, glm::vec3(0.0f, -1.0f, 0.0f));
		cameraPosition_ += cameraVelocity * deltaTime * 2.0f * cameraRotation_;

		previousCursorPosition_ = cursorPosition_;

		// Draw new frame
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glUseProgram(shaderProgram_);

		glm::mat4 cameraView = glm::translate(glm::toMat4(cameraRotation_), -cameraPosition_);
		glm::mat4 modelViewProjection = perspectiveProjection_ * cameraView;

		glUniformMatrix4fv(glGetUniformLocation(shaderProgram_, "u_modelViewProjection"), 1, false, glm::value_ptr(modelViewProjection));

		navMesh_->Draw();

		glfwSwapBuffers(glfwWindow_);
		glfwPollEvents();

		// Reset previous cursor position if started dragging camera to avoid a sudden jolt in rotation
		if (resetPreviousCursorPosition_)
		{
			double cursorX;
			double cursorY;

			glfwGetCursorPos(glfwWindow_, &cursorX, &cursorY);
			previousCursorPosition_ = glm::vec2(cursorX, cursorY);

			resetPreviousCursorPosition_ = false;
		}
	}
}

bool Window::IsValid()
{
	return valid_;
}

glm::mat4 Window::GetPerspectiveProjection(const int& width, const int& height)
{
	return glm::perspective(glm::quarter_pi<float>(), width / (float)height, 0.1f, 400.0f);
}

void Window::CursorPosCallback(GLFWwindow* glfwWindow, double xpos, double ypos)
{
	Window* window = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));

	// Update cursorPosition_
	window->cursorPosition_ = glm::vec2(xpos, ypos);
}

void Window::KeyCallback(GLFWwindow* glfwWindow, int key, int scancode, int action, int mods)
{
	Window* window = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));

	// Handle user key presses
	switch (key)
	{
	case GLFW_KEY_W:
		if (action == GLFW_PRESS)
		{
			window->forwardPressed_ = true;
		}
		else if (action == GLFW_RELEASE)
		{
			window->forwardPressed_ = false;
		}

		break;

	case GLFW_KEY_A:
		if (action == GLFW_PRESS)
		{
			window->leftPressed_ = true;
		}
		else if (action == GLFW_RELEASE)
		{
			window->leftPressed_ = false;
		}

		break;

	case GLFW_KEY_S:
		if (action == GLFW_PRESS)
		{
			window->backwardPressed_ = true;
		}
		else if (action == GLFW_RELEASE)
		{
			window->backwardPressed_ = false;
		}

		break;

	case GLFW_KEY_D:
		if (action == GLFW_PRESS)
		{
			window->rightPressed_ = true;
		}
		else if (action == GLFW_RELEASE)
		{
			window->rightPressed_ = false;
		}

		break;

	case GLFW_KEY_LEFT_CONTROL:
		if (action == GLFW_PRESS)
		{
			window->downwardPressed_ = true;
		}
		else if (action == GLFW_RELEASE)
		{
			window->downwardPressed_ = false;
		}

		break;

	case GLFW_KEY_LEFT_SHIFT:
		if (action == GLFW_PRESS)
		{
			window->shiftPressed_ = true;
		}
		else if (action == GLFW_RELEASE)
		{
			window->shiftPressed_ = false;
		}

		break;

	case GLFW_KEY_SPACE:
		if (action == GLFW_PRESS)
		{
			window->upwardPressed_ = true;
		}
		else if (action == GLFW_RELEASE)
		{
			window->upwardPressed_ = false;
		}

		break;

	case GLFW_KEY_Q:
		if (action == GLFW_PRESS)
		{
			// Change whether to select the start triangle or end triangle.
			// 0 = start triangle
			// 1 = end triangle
			window->triangleToSelect_++;

			if (window->triangleToSelect_ >= 2)
			{
				window->triangleToSelect_ = 0;
			}
		}

		break;

	case GLFW_KEY_ESCAPE:
		if (action == GLFW_PRESS)
		{
			glfwSetWindowShouldClose(glfwWindow, GLFW_TRUE);
		}

		break;
	}
}

void Window::MouseButtonCallback(GLFWwindow* glfwWindow, int button, int action, int mods)
{
	Window* window = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));

	// Handle user mouse button presses
	switch (button)
	{
	case GLFW_MOUSE_BUTTON_RIGHT:
		if (action == GLFW_PRESS)
		{
			glfwSetInputMode(glfwWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

			window->resetPreviousCursorPosition_ = true;
			window->rightMousePressed_ = true;
		}
		else if (action == GLFW_RELEASE)
		{
			glfwSetInputMode(glfwWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

			window->resetPreviousCursorPosition_ = false;
			window->rightMousePressed_ = false;
		}

		break;

	case GLFW_MOUSE_BUTTON_LEFT:
		if (action == GLFW_PRESS)
		{
			if (!window->leftMousePressed_)
			{
				int windowWidth;
				int windowHeight;

				glfwGetWindowSize(glfwWindow, &windowWidth, &windowHeight);

				glm::mat4 modelViewProjection = window->perspectiveProjection_ * glm::translate(glm::toMat4(window->cameraRotation_), -window->cameraPosition_);

				// Calculate which triangle the user clicked and update selected triangles
				uint32_t clickedTriangleIndex = window->navMesh_->GetTriangleIndexAtCursorPos(
					glm::vec2((window->cursorPosition_.x - windowWidth / 2) * 2 / windowWidth,
					-(window->cursorPosition_.y - windowHeight / 2) * 2 / windowHeight), modelViewProjection);

				if (clickedTriangleIndex != -1)
				{
					window->selectedTriangles_[window->triangleToSelect_] = clickedTriangleIndex;

					window->navMesh_->GetPathBetweenCellIndices(window->selectedTriangles_[0], window->selectedTriangles_[1]);
				}
			}

			window->leftMousePressed_ = true;
		}
		else if (action == GLFW_RELEASE)
		{
			window->leftMousePressed_ = false;
		}

		break;
	}
}

void Window::WindowSizeCallback(GLFWwindow* glfwWindow, int width, int height)
{
	Window* window = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));

	// Change perspective projection matrix and viewport to match new window size
	window->perspectiveProjection_ = GetPerspectiveProjection(width, height);
	glViewport(0, 0, width, height);
}
