#include "attribute-less-mesh.h"

using namespace regen;

AttributeLessMesh::AttributeLessMesh(uint32_t numVertices)
		: Mesh(GL_POINTS, BufferUpdateFlags::NEVER) {
	set_numVertices(numVertices);
}
