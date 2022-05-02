#ifndef NAVMESH_H
#define NAVMESH_H

// This class implements the navigation mesh data structure and the A*
// pathfinding algorithm that is used to navigate within it.
// Original author: Arttu Leppäaho arttu.leppaaho@hotmail.com
// Date: 4.2022
// Licensed under the MIT license

#include "cell.h"

#include <glm/glm.hpp>
#include <vector>
#include <forward_list>
#include <map>
#include <set>
#include <functional>

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
	std::forward_list<Cell*> GetPathBetweenCellIndices(const uint32_t& startCellIndex, const uint32_t& endCellIndex);
	// Returns true if the NavMesh has initialized correctly and can be used.
	bool IsValid() const;
protected:
	// Represents a node along a pathfinding path.
	struct PathNode
	{
		// The previous node leading towards the start of the path.
		PathNode* previousNode;
		// The navigation mesh cell that this PathNode represents.
		Cell* cell;
		// The distance to the start of the path from this node.
		float distanceToStart;
		// The shortest possible path length that can be found through this node.
		float shortestPotentialPathLength;
	};

	using CellPriorityQueue = std::set<PathNode*, std::function<bool(PathNode*, PathNode*)>>;

	struct PathfindingTask
	{
		// The start cell of this pathfinding task.
		Cell* startCell;
		// The end goal cell of this pathfinding task.
		Cell* endCell;
		// The PathNode corresponding to the end cell if one has been found.
		PathNode* endPathNode;
		// The PathNodes that have been created for this pathfinding task mapped to the corresponding Cells.
		std::map<Cell*, PathNode> cellsToPathNodes;
		// A priority queue of PathNodes that have been discovered but not yet processed.
		CellPriorityQueue discovered;
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

	// Creates a PathNode for the given Cell and adds it to the given map and priority queue.
	void CreateOrUpdatePathNodeForCell(Cell* cell, PathNode* previousPathNode, PathfindingTask& task) const;
	// Collects the nodes of a finished PathfindingTask.
	std::forward_list<Cell*> NavMesh::CollectPath(PathfindingTask& task) const;
	// A* algorithm implementation for finding a path between the given cells.
	std::forward_list<Cell*> AStar(PathfindingTask& task) const;
};

#endif // NAVMESH_H
