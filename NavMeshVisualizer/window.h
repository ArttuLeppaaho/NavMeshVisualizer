#ifndef WINDOW_H
#define WINDOW_H

// This class handles the window that renders the navigation mesh. This
// includes initializing GLFW and GLAD, handling input, handling window
// events, controlling the camera and initializing OpenGL basics.
// Author: Arttu Leppäaho arttu.leppaaho@hotmail.com
// Date: 4.2022

#include <glad/glad.h> 
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include <vector>

class NavMesh;

class Window
{
public:
	Window();
	~Window();

	// Gets the OpenGL shader program used for all rendering in the program.
	static uint32_t GetShaderProgram();

	// Starts the Window's event loop.
	void Run();
	// Returns true if the Window has initialized correctly and can be used.
	bool IsValid();

protected:
	// The OpenGL shader program used for all rendering in the program.
	static uint32_t shaderProgram_;

	// True if the window has initialized correctly and can be used.
	bool valid_;

	// The GLFW window used by this Window.
	GLFWwindow* glfwWindow_;
	// The navigation mesh that this window displays and interacts with.
	NavMesh* navMesh_;

	// The perspective projection matrix used when rendering the camera view.
	glm::mat4 perspectiveProjection_;

	// The current position of the cursor.
	glm::vec2 cursorPosition_;
	// The position of the cursor on the previous frame.
	glm::vec2 previousCursorPosition_;

	// The 3D position of the camera.
	glm::vec3 cameraPosition_;
	// The 3D rotation of the camera.
	glm::quat cameraRotation_;

	// True if the previous cursor position should be ignored this frame because
	// the user started dragging the camera.
	bool resetPreviousCursorPosition_;

	// True if the mentioned control is currently pressed.
	bool forwardPressed_;
	bool backwardPressed_;
	bool leftPressed_;
	bool rightPressed_;
	bool upwardPressed_;
	bool downwardPressed_;
	bool shiftPressed_;
	bool rightMousePressed_;
	bool leftMousePressed_;

	// The selected triangle indices in the navigation mesh.
	uint32_t selectedTriangles_[2];
	// Which triangle index to select when clicking.
	uint8_t triangleToSelect_;

	// Calculates a perspective projection matrix for the given viewport width and height.
	static glm::mat4 GetPerspectiveProjection(const int& width, const int& height);

	// GLFW callbacks for different GLFW window events.
	static void CursorPosCallback(GLFWwindow* glfwWindow, double xpos, double ypos);
	static void KeyCallback(GLFWwindow* glfwWindow, int key, int scancode, int action, int mods);
	static void MouseButtonCallback(GLFWwindow* glfwWindow, int button, int action, int mods);
	static void WindowSizeCallback(GLFWwindow* glfwWindow, int width, int height);
};

#endif // WINDOW_H
