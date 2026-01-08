#pragma once
#include <vector>
#include <physics/transform.hpp>

namespace phe::physics {

	struct Mesh {
		std::vector<float> vertices;
		std::vector<unsigned int> indices;

		bool hasRenderContext = false;
		unsigned int vao, vbo, ebo;
	};

	int getNumVerts(Mesh& m);
	glm::vec3 getVertex(Mesh& m, math::Transform trans, int i);
	Mesh createCubeMesh(glm::vec3 size, glm::vec3 rgb);

}
