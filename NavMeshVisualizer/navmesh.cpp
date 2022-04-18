// Original author: Arttu Leppäaho arttu.leppaaho@hotmail.com
// Date: 4.2022
// Licensed under the MIT license

#include "navmesh.h"
#include "navmeshrenderer.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include "glm/gtx/projection.hpp"

#include <iostream>

const glm::vec4 PATH_TRIANGLES_COLOR = glm::vec4(0.1f, 0.6f, 1.0f, 1.0f);
const glm::vec4 PROCESSED_TRIANGLES_COLOR = glm::vec4(1.0f, 0.6f, 0.1f, 1.0f);
const glm::vec4 START_TRIANGLE_COLOR = glm::vec4(0.1f, 1.0f, 0.3f, 1.0f);
const glm::vec4 END_TRIANGLE_COLOR = glm::vec4(1.0f, 0.1f, 0.3f, 1.0f);

NavMesh::NavMesh(const std::string& filePath) : valid_(true)
{
	// Construct the navigation mesh from the .obj file

	tinyobj::ObjReaderConfig readerConfig;
	tinyobj::ObjReader reader;

	readerConfig.triangulate = true;

	// Load .obj file. Abort if there's an error
	if (!reader.ParseFromFile(filePath, readerConfig))
	{
		if (!reader.Error().empty())
		{
			std::cerr << "TinyObjReader: " << reader.Error();
		}

		valid_ = false;

		return;
	}

	// We could read warnings here but then the program would always complain
	// about missing materials even though we don't need materials
	/*if (!reader.Warning().empty())
	{
		std::cout << "TinyObjReader: " << reader.Warning();
	}*/

	const tinyobj::attrib_t& attrib = reader.GetAttrib();
	const std::vector<tinyobj::shape_t>& shapes = reader.GetShapes();
	std::vector<std::forward_list<Cell*>> cellsPerVertex;

	vertices_.resize(attrib.vertices.size() / 3);

	// Read all vertices from the .obj file
	for (size_t i = 0; i < attrib.vertices.size() / 3; i++)
	{
		vertices_[i] =
		{
			attrib.vertices[3 * i + 0],
			attrib.vertices[3 * i + 1],
			attrib.vertices[3 * i + 2]
		};

		cellsPerVertex.push_back({});
	}

	// Calculate total cell count and reserve space accordingly in the cells_ vector
	size_t totalCells = 0;

	for (size_t i = 0; i < shapes.size(); i++)
	{
		totalCells += shapes[i].mesh.num_face_vertices.size();
	}

	cells_.resize(totalCells);

	size_t previousFaceCounts = 0;

	// Process each shape (object) in the .obj file
	for (size_t i = 0; i < shapes.size(); i++)
	{
		size_t indexOffset = 0;
		const size_t faceCount = shapes[i].mesh.num_face_vertices.size();

		// Create a navigation mesh cell from each triangle of the shape
		for (size_t j = 0; j < faceCount; j++)
		{
			Cell& thisCell = cells_[previousFaceCounts + j];
			thisCell.index = previousFaceCounts + j;

			glm::vec3 vertexPositionSum(0.0f);

			thisCell.vertexIndices.resize(3);

			// Process each vertex index of the triangle
			for (size_t k = 0; k < 3; k++)
			{
				tinyobj::index_t idx = shapes[i].mesh.indices[indexOffset + k];

				triangleVertexIndices_.push_back(idx.vertex_index);

				thisCell.vertexIndices[k] = idx.vertex_index;
				vertexPositionSum += vertices_[idx.vertex_index];

				// Track which cells use which vertices for connecting cells later
				cellsPerVertex[idx.vertex_index].push_front(&thisCell);
			}

			thisCell.centerPosition = vertexPositionSum / 3.0f;

			indexOffset += 3;

			// Find cells that share an edge with this one and connect them.
			// Cells share an edge if they share two vertices
			for (uint32_t vertexIndex : thisCell.vertexIndices)
			{
				// Get other cells that share this vertex
				for (Cell* otherCell : cellsPerVertex[vertexIndex])
				{
					// Check if there are other shared vertices in the other cell
					for (uint32_t otherCellVertexIndex : otherCell->vertexIndices)
					{
						// Don't count the same vertex as a second one
						if (otherCellVertexIndex == vertexIndex)
						{
							continue;
						}

						for (uint32_t thisCellOtherVertexIndex : thisCell.vertexIndices)
						{
							if (thisCellOtherVertexIndex == otherCellVertexIndex)
							{
								// Check that the cells are not already connected
								bool alreadyConnected = false;

								for (Cell* connectedCell : thisCell.connectedCells)
								{
									if (connectedCell == otherCell)
									{
										alreadyConnected = true;
										break;
									}
								}

								if (alreadyConnected)
								{
									continue;
								}

								// These cells share two vertices and are not yet connected.
								// Connect them
								thisCell.connectedCells.push_front(otherCell);
								otherCell->connectedCells.push_front(&thisCell);
							}
						}
					}
				}
			}
		}

		previousFaceCounts += faceCount;
	}

	// Create renderer for drawing the navigation mesh
	renderer_ = new NavMeshRenderer(vertices_.data(), vertices_.size(), triangleVertexIndices_.data(), triangleVertexIndices_.size());
}

NavMesh::~NavMesh()
{
	delete renderer_;
}

void NavMesh::Draw() const
{
	renderer_->Render();
}

uint32_t NavMesh::GetTriangleIndexAtCursorPos(const glm::vec2& cursorPos, const glm::mat4& modelViewProjection)
{
	// This is a fairly simple but inefficient way to find the closest triangle at the cursor.
	// This method has O(n) complexity as it must go through every triangle in the navigation
	// mesh to determine the best triangle. A better method would be to use a bounding volume
	// hierarchy to quickly eliminate parts of the navigation mesh from the search.

	std::vector<glm::vec3> viewportSpaceVertices;
	viewportSpaceVertices.resize(vertices_.size());

	// First transform all vertices to viewport space
	for (size_t i = 0; i < vertices_.size(); i++)
	{
		glm::vec4 transformedVertex = modelViewProjection * glm::vec4(vertices_[i], 1);

		viewportSpaceVertices[i] = glm::vec3(transformedVertex.x, transformedVertex.y, transformedVertex.z) / transformedVertex.w;
	}

	uint32_t closestTriangleIndex = -1;
	float minimumTriangleDepth = 1;

	// Then analyze each triangle
	for (size_t i = 0; i < triangleVertexIndices_.size() / 3; i++)
	{
		glm::vec3 triangleVertex0 = viewportSpaceVertices[triangleVertexIndices_[i * 3 + 0]];
		glm::vec3 triangleVertex1 = viewportSpaceVertices[triangleVertexIndices_[i * 3 + 1]];
		glm::vec3 triangleVertex2 = viewportSpaceVertices[triangleVertexIndices_[i * 3 + 2]];

		// Ignore triangles behind the camera
		if (triangleVertex0.z > 1.0f || triangleVertex1.z > 1.0f || triangleVertex2.z > 1.0f)
		{
			continue;
		}

		glm::vec2 triangleEdge0 = glm::vec2(triangleVertex1.x - triangleVertex0.x, triangleVertex1.y - triangleVertex0.y);
		glm::vec2 triangleEdge1 = glm::vec2(triangleVertex2.x - triangleVertex0.x, triangleVertex2.y - triangleVertex0.y);
		glm::vec2 vertex0ToCursor = glm::vec2(cursorPos.x - triangleVertex0.x, cursorPos.y - triangleVertex0.y);

		// Calculate barycentric coordinates for cursor point on triangle
		float dot00 = glm::dot(triangleEdge0, triangleEdge0);
		float dot01 = glm::dot(triangleEdge0, triangleEdge1);
		float dot11 = glm::dot(triangleEdge1, triangleEdge1);
		float dotC0 = glm::dot(vertex0ToCursor, triangleEdge0);
		float dotC1 = glm::dot(vertex0ToCursor, triangleEdge1);

		float denominator = dot00 * dot11 - dot01 * dot01;
		float vCoord = (dot11 * dotC0 - dot01 * dotC1) / denominator;
		float wCoord = (dot00 * dotC1 - dot01 * dotC0) / denominator;
		float uCoord = 1.0f - vCoord - wCoord;

		// If point is within this triangle
		if (uCoord >= 0.0f && vCoord >= 0.0f && wCoord >= 0.0f)
		{
			float triangleDepthAtCursor = uCoord * triangleVertex0.z + vCoord * triangleVertex1.z + wCoord * triangleVertex2.z;

			if (triangleDepthAtCursor < minimumTriangleDepth)
			{
				// If triangle is closer to the camera than the previous best triangle,
				// mark this as the new best triangle
				minimumTriangleDepth = triangleDepthAtCursor;
				closestTriangleIndex = i;
			}
		}
	}

	// Return the closest triangle at the clicked position
	return closestTriangleIndex;
}

std::forward_list<Cell*> NavMesh::GetPathBetweenCellIndices(const uint32_t& startCellIndex, const uint32_t& endCellIndex)
{
	// Get the cells corresponding to these cell indices
	Cell* startCell = startCellIndex < cells_.size() ? &cells_[startCellIndex] : nullptr;
	Cell* endCell = endCellIndex < cells_.size() ? &cells_[endCellIndex] : nullptr;

	// Create a pathfinding task struct to hold data for this pathfinding query
	PathfindingTask task = { startCell, endCell, nullptr,{}, CellPriorityQueue(ComparePathNodes) };

	return AStar(task);
}

bool NavMesh::IsValid() const
{
	return valid_;
}

bool NavMesh::ComparePathNodes(PathNode* nodeA, PathNode* nodeB)
{
	return nodeA->shortestPotentialPathLength < nodeB->shortestPotentialPathLength;
}

float NavMesh::GetDistanceBetween(Cell* cellA, Cell* cellB)
{
	return glm::length(cellA->centerPosition - cellB->centerPosition);
}

void NavMesh::CreateOrUpdatePathNodeForCell(Cell* cell, PathNode* previousPathNode, PathfindingTask& task) const
{
	// Calculate the distance from the start to the connected cell. If
	// previousPathNode is nullptr then this is the first cell and the distance
	// is zero
	float cellDistanceToStart = previousPathNode != nullptr
		? previousPathNode->distanceToStart + GetDistanceBetween(previousPathNode->correspondingCell, cell)
		: 0.0f;

	// Try to find an existing PathNode for the cell
	auto existingPathNodeIt = task.cellsToPathNodes.find(cell);

	// If no existing PathNode found
	if (existingPathNodeIt == task.cellsToPathNodes.end())
	{
		// Create a new PathNode and add it to the priority queue
		task.cellsToPathNodes.insert({ cell,
		{
			previousPathNode,
			cell,
			cellDistanceToStart,
			cellDistanceToStart + GetDistanceBetween(cell, task.endCell)
		}});

		task.discovered.insert(&task.cellsToPathNodes.at(cell));
	}
	else
	{
		// PathNode already exists, fetch it
		PathNode& connectedPathNode = existingPathNodeIt->second;

		// If the distance to the start is shorter through this cell,
		// update the PathNode's distances and previous node
		if (cellDistanceToStart < connectedPathNode.distanceToStart)
		{
			// If the PathNode is currently in the priority queue,
			// remove it and insert it back to update its priority
			auto connectedPathNodeInDiscoveredIt = task.discovered.find(&connectedPathNode);
			bool priorityQueueNeedsRefreshing = false;

			if (connectedPathNodeInDiscoveredIt != task.discovered.end())
			{
				task.discovered.erase(connectedPathNodeInDiscoveredIt);
				priorityQueueNeedsRefreshing = true;
			}

			connectedPathNode.previousNode = previousPathNode;
			connectedPathNode.distanceToStart = cellDistanceToStart;
			connectedPathNode.shortestPotentialPathLength = cellDistanceToStart + GetDistanceBetween(cell, task.endCell);

			if (priorityQueueNeedsRefreshing)
			{
				task.discovered.insert(&connectedPathNode);
			}
		}
	}
}

std::forward_list<Cell*> NavMesh::CollectPath(PathfindingTask& task) const
{
	std::forward_list<Cell*> pathCells;
	std::vector<uint32_t> pathCellIndices;
	std::vector<uint32_t> processedCellIndices;

	// Gather all processed cells for visualization
	for (const std::pair<Cell*, PathNode>& pair : task.cellsToPathNodes)
	{
		// Don't add discovered cells that have not been processed yet
		if (task.discovered.find((PathNode*)&pair.second) != task.discovered.end())
		{
			continue;
		}

		processedCellIndices.push_back(pair.first->index);
	}

	// Gather path cells
	PathNode* pathNode = task.endPathNode;

	while (pathNode != nullptr)
	{
		pathCells.push_front(pathNode->correspondingCell);
		pathCellIndices.push_back(pathNode->correspondingCell->index);

		pathNode = pathNode->previousNode;
	}

	// Highlight interesting triangles with different colors
	renderer_->ClearHighlightedTriangles();
	renderer_->HighlightTriangles(processedCellIndices.data(), processedCellIndices.size(), PROCESSED_TRIANGLES_COLOR);
	renderer_->HighlightTriangles(pathCellIndices.data(), pathCellIndices.size(), PATH_TRIANGLES_COLOR);

	if (task.startCell != nullptr)
	{
		renderer_->HighlightTriangles(&task.startCell->index, 1, START_TRIANGLE_COLOR);
	}

	if (task.endCell != nullptr)
	{
		renderer_->HighlightTriangles(&task.endCell->index, 1, END_TRIANGLE_COLOR);
	}

	return pathCells;
}

std::forward_list<Cell*> NavMesh::AStar(PathfindingTask& task) const
{
	if (task.startCell != nullptr && task.endCell != nullptr)
	{
		CreateOrUpdatePathNodeForCell(task.startCell, nullptr, task);
	}

	// Process PathNodes until no new ones are discovered and the entire search
	// space has been explored
	while (!task.discovered.empty())
	{
		// Pop the highest priority unprocessed node from the priority queue
		auto highestPriorityUnprocessedNode = task.discovered.begin();
		PathNode* nodeToProcess = *highestPriorityUnprocessedNode;
		task.discovered.erase(highestPriorityUnprocessedNode);

		// If the node is the goal node, the path has been found: stop the loop
		if (nodeToProcess->correspondingCell == task.endCell)
		{
			task.endPathNode = nodeToProcess;
			break;
		}

		// Process each cell connected to the current cell
		for (Cell* connectedCell : nodeToProcess->correspondingCell->connectedCells)
		{
			CreateOrUpdatePathNodeForCell(connectedCell, nodeToProcess, task);
		}
	}

	// Pathfinding finished, collect the discovered path if one was found
	return CollectPath(task);
}
