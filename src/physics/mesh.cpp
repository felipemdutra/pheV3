#include "physics/mesh.hpp"

#include <stdexcept>

namespace phe::physics {

	int getNumVerts(Mesh& m) {
		return m.vertices.size() / 6;
	}

	glm::vec3 getVertex(Mesh& m, math::Transform trans, int i) {
		if (i > m.vertices.size() || i < 0) {
			throw std::out_of_range("Vertex index is out of range");
		}

		int vertexStart = i * 6;

		glm::vec3 localVertex(
			m.vertices[vertexStart],
			m.vertices[vertexStart + 1],
			m.vertices[vertexStart + 2]
		);

		return glm::vec3(trans.rotation * glm::vec4(localVertex, 1.0f)) + trans.translation;
	}

	Mesh createCubeMesh(glm::vec3 size, glm::vec3 rgb) {
		float hx = size.x * 0.5;
		float hy = size.y * 0.5;
		float hz = size.z * 0.5;

		float r = rgb.x;
		float g = rgb.y;
		float b = rgb.b;

		std::vector<float> cubeVertices = {
			-hx, -hy,  hz,  r, g, b,
			 hx, -hy,  hz,  r, g, b,
			 hx,  hy,  hz,  r, g, b,
			-hx,  hy,  hz,  r, g, b,

			-hx, -hy, -hz,  r, g, b,
			-hx,  hy, -hz,  r, g, b,
			 hx,  hy, -hz,  r, g, b,
			 hx, -hy, -hz,  r, g, b,

			-hx, -hy, -hz,  r, g, b,
			-hx, -hy,  hz,  r, g, b,
			-hx,  hy,  hz,  r, g, b,
			-hx,  hy, -hz,  r, g, b,

			 hx, -hy, -hz,  r, g, b,
			 hx,  hy, -hz,  r, g, b,
			 hx,  hy,  hz,  r, g, b,
			 hx, -hy,  hz,  r, g, b,

			-hx,  hy, -hz,  r, g, b,
			-hx,  hy,  hz,  r, g, b,
			 hx,  hy,  hz,  r, g, b,
			 hx,  hy, -hz,  r, g, b,

			 hx, -hy, -hz,  r, g, b,
			 hx, -hy,  hz,  r, g, b,
			-hx, -hy,  hz,  r, g, b,
			-hx, -hy, -hz,  r, g, b,
		};


		std::vector<unsigned int> cubeIndices = {
			0, 1, 2, 2, 3, 0,
			4, 5, 6, 6, 7, 4,
			8, 9, 10, 10, 11, 8,
			12, 13, 14, 14, 15, 12,
			16, 17, 18, 18, 19, 16,
			20, 21, 22, 22, 23, 20
		};

		return {cubeVertices, cubeIndices};
	}

}
