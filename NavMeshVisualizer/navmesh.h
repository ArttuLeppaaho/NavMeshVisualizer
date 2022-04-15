#ifndef NAVMESH_H
#define NAVMESH_H

// This class implements the navigation mesh data structure and the A*
// pathfinding algorithm that is used to navigate within it.
// Author: Arttu Lepp�aho arttu.leppaaho@hotmail.com
// Date: 4.2022

#include "cell.h"

#include <glm/glm.hpp>
#include <vector>
#include <forward_list>

class NavMeshRenderer;

class NavMesh
{
public:
	NavMesh(const std::string& filePath);
	~NavMesh();

	// Draws the NavMesh.
	void Draw() const;
	// Gets the triangle at the given cursor position and using the given model-view-projection matrix as the camera.
	uint32_t GetTriangleIndexAtCursorPos(const glm::vec2& cursorPos, const glm::mat4& modelViewProjection);
	// Calculates the shortest path between the given cells if one exists. Returns the cells that make up the path.
	std::forward_list<Cell*> GetPathBetweenCellIndices(const uint32_t& from, const uint32_t& to);
	// Returns true if the NavMesh has initialized correctly and can be used.
	bool IsValid() const;
protected:
	// Represents a node along a pathfinding path.
	struct PathNode
	{
		// The previous node leading towards the start of the path.
		PathNode* previousNode;
		// The navigation mesh cell that this PathNode represents.
		Cell* correspondingCell;
		// The distance to the start of the path from this node.
		float distanceToStart;
		// The shortest possible path length that can be found through this node.
		float shortestPotentialPathLength;
	};

	// The vertices this navigation mesh consists of.
	std::vector<glm::vec3> vertices_;
	// The vertex indices of the triangles that this navigation mesh consists of.
	std::vector<uint32_t> triangleVertexIndices_;
	// The cells that this navigation mesh consists of.
	std::vector<Cell> cells_;

	// The renderer to use for drawing this NavMesh.
	NavMeshRenderer* renderer_;

	// True if the NavMesh has initialized correctly and can be used.
	bool valid_;

	// Compares the shortest possible path lengths of the given PathNodes. Used
	// to prioritize nodes in the A* algorithm.
	static bool ComparePathNodes(PathNode* nodeA, PathNode* nodeB);
	// Calculates the distance between the given cells.
	static float GetDistanceBetween(Cell* cellA, Cell* cellB);
	// A* algorithm implementation for finding a path between the given cells.
	std::forward_list<Cell*> AStar(Cell* from, Cell* to) const;
};

#endif // NAVMESH_H
