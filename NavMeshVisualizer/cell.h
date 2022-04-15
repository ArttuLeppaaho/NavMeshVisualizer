#ifndef CELL_H
#define CELL_H

// This struct represents a navigation mesh cell. Cells always belong to a
// NavMesh and they're not meant to be used alone.
// Author: Arttu Leppäaho arttu.leppaaho@hotmail.com
// Date: 4.2022

#include <glm/glm.hpp>

#include <vector>
#include <forward_list>

struct Cell
{
	// The index of this cell in its NavMesh's cells_ vector.
	uint32_t index;
	// The indices of the vertices that make up this cell in its NavMesh.
	std::vector<uint32_t> vertexIndices;
	// The position of this cell's center in its NavMesh.
	glm::vec3 centerPosition;
	// The cells connected to this cell in its NavMesh.
	std::forward_list<Cell*> connectedCells;
};

#endif // CELL_H
