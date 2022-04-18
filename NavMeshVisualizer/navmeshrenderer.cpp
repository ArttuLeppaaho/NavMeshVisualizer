// Original author: Arttu Leppäaho arttu.leppaaho@hotmail.com
// Date: 4.2022
// Licensed under the MIT license

#include "navmeshrenderer.h"
#include "window.h"

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>

#include <vector>

NavMeshRenderer::NavMeshRenderer(glm::vec3* vertices, const size_t& vertexCount,
	uint32_t* triangleVertexIndices, const size_t& triangleVertexIndexCount)
	: triangleVertexIndexCount_(triangleVertexIndexCount), triangleVertexIndices_(triangleVertexIndices)
{
	// Create OpenGL buffers and store mesh data in them
	glGenVertexArrays(1, &vertexArrayObject_);
	glBindVertexArray(vertexArrayObject_);

	glGenBuffers(1, &vertexBufferObject_);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject_);
	glBufferData(GL_ARRAY_BUFFER, vertexCount * sizeof(glm::vec3), vertices, GL_STATIC_DRAW);

	glGenBuffers(1, &elementBufferObject_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBufferObject_);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangleVertexIndexCount_ * sizeof(uint32_t), triangleVertexIndices, GL_STATIC_DRAW);

	glGenBuffers(1, &highlightedElementBufferObject_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, highlightedElementBufferObject_);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangleVertexIndexCount_ * sizeof(uint32_t), nullptr, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

NavMeshRenderer::~NavMeshRenderer()
{
	glDeleteBuffers(1, &vertexBufferObject_);
	glDeleteVertexArrays(1, &vertexArrayObject_);
	glDeleteBuffers(1, &elementBufferObject_);
	glDeleteBuffers(1, &highlightedElementBufferObject_);
}

void NavMeshRenderer::Render() const
{
	// Bind data to render
	glBindVertexArray(vertexArrayObject_);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	int areaIndex = 0;

	// Draw highlighted areas
	for (const HighlightedArea& highlightedArea : highlightedAreas_)
	{
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, highlightedElementBufferObject_);
		glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, highlightedArea.triangleIndices.size() * sizeof(uint32_t), highlightedArea.triangleIndices.data());

		// Add a depth offset so that different color areas don't overlap
		glUniform1f(glGetUniformLocation(Window::GetShaderProgram(), "u_depthOffset"), 0.00001f + 0.0000001f * areaIndex);
		// Set area color
		glUniform4fv(glGetUniformLocation(Window::GetShaderProgram(), "u_color"), 1, glm::value_ptr(highlightedArea.color));

		// Render
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, highlightedElementBufferObject_);
		glDrawElements(GL_TRIANGLES, highlightedArea.triangleIndices.size(), GL_UNSIGNED_INT, 0);

		areaIndex++;
	}

	// Draw whole mesh
	glUniform1f(glGetUniformLocation(Window::GetShaderProgram(), "u_depthOffset"), 0.00001f + 0.0000001f * areaIndex);
	glUniform4f(glGetUniformLocation(Window::GetShaderProgram(), "u_color"), 0.6f, 0.6f, 0.6f, 1.0f);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBufferObject_);
	glDrawElements(GL_TRIANGLES, triangleVertexIndexCount_, GL_UNSIGNED_INT, 0);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// Draw wireframe over mesh
	glUniform1f(glGetUniformLocation(Window::GetShaderProgram(), "u_depthOffset"), 0.0f);
	glUniform4f(glGetUniformLocation(Window::GetShaderProgram(), "u_color"), 1.0f, 1.0f, 1.0f, 1.0f);

	glDrawElements(GL_TRIANGLES, triangleVertexIndexCount_, GL_UNSIGNED_INT, 0);

	// Unbind data
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void NavMeshRenderer::HighlightTriangles(uint32_t* triangleIndicesToHighlight, const size_t& highlightedTriangleCount, const glm::vec4& color)
{
	highlightedAreas_.push_front({ {}, color});

	std::vector<uint32_t>& highlightedTriangleIndices = highlightedAreas_.front().triangleIndices;

	highlightedTriangleIndices.resize(highlightedTriangleCount * 3);

	// Get vertex indices for the given triangle indicess
	for (size_t i = 0; i < highlightedTriangleCount; i++)
	{
		highlightedTriangleIndices[i * 3 + 0] = triangleVertexIndices_[triangleIndicesToHighlight[i] * 3 + 0];
		highlightedTriangleIndices[i * 3 + 1] = triangleVertexIndices_[triangleIndicesToHighlight[i] * 3 + 1];
		highlightedTriangleIndices[i * 3 + 2] = triangleVertexIndices_[triangleIndicesToHighlight[i] * 3 + 2];
	}
}

void NavMeshRenderer::ClearHighlightedTriangles()
{
	highlightedAreas_.clear();
}
