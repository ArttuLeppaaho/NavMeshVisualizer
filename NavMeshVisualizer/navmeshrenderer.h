#ifndef NAVMESHRENDERER_H
#define NAVMESHRENDERER_H

// This class renders the navigation mesh and allows highlighting specific
// triangles with different colors.
// Author: Arttu Leppäaho arttu.leppaaho@hotmail.com
// Date: 4.2022

#include <glm/glm.hpp>
#include <vector>
#include <forward_list>

class NavMeshRenderer
{
public:
	NavMeshRenderer(glm::vec3* vertices, const size_t& vertexCount,
		uint32_t* triangleVertexIndices, const size_t& triangleVertexIndexCount);
	~NavMeshRenderer();

	// Renders the NavMesh.
	void Render() const;
	// Highlights the given group of triangles with the given color until highlights are cleared.
	void HighlightTriangles(uint32_t* triangleIndicesToHighlight, const size_t& highlightedTriangleCount, const glm::vec4& color);
	// Clears all currently active highlighted triangles.
	void ClearHighlightedTriangles();

protected:
	// Stores the details of a highlighted area.
	struct HighlightedArea
	{
		// The triangle indices that make up this highlighted area.
		std::vector<uint32_t> triangleIndices;
		// The color of this highlighted area.
		glm::vec4 color;
	};

	// The OpenGL vertex array object used to store the mesh data for rendering.
	uint32_t vertexArrayObject_;
	// The OpenGL vertex buffer object used to store the mesh data for rendering.
	uint32_t vertexBufferObject_;
	// The OpenGL element buffer object used to store the mesh data for rendering.
	uint32_t elementBufferObject_;
	// The OpenGL element buffer object used to store the highlighted mesh data for rendering.
	uint32_t highlightedElementBufferObject_;

	// How many vertex indices the triangles of the mesh use.
	uint32_t triangleVertexIndexCount_;

	// A pointer to the NavMesh's triangle vertex index data.
	uint32_t* triangleVertexIndices_;

	// Currently highlighted areas of the mesh.
	std::forward_list<HighlightedArea> highlightedAreas_;
};

#endif // NAVMESHRENDERER_H
