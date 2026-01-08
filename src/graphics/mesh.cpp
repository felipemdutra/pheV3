#include <GL/glew.h>
#include <stdexcept>
#include <glm/gtc/quaternion.hpp>

#include "graphics/mesh.hpp"
#include "../../include/physics/transform.hpp"

namespace phe::graphics {

void prepareMeshRenderContext(physics::Mesh& m) {
    glGenVertexArrays(1, &m.vao);
    glBindVertexArray(m.vao);

    glGenBuffers(1, &m.vbo);
    glGenBuffers(1, &m.ebo);

    glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ebo);

    glBufferData(GL_ARRAY_BUFFER, m.vertices.size() * sizeof(float),
                m.vertices.data(), GL_STATIC_DRAW);

    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m.indices.size() * 
                sizeof(unsigned int), m.indices.data(), GL_STATIC_DRAW);

    // Position in local space
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 
            (void*)0);
    glEnableVertexAttribArray(0);

    // Color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 
            (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void destroyMesh(physics::Mesh& m) {
	if (!m.hasRenderContext) {
		return;
	}

    glDeleteVertexArrays(1, &m.vao);
    glDeleteBuffers(1, &m.vbo);
    glDeleteBuffers(1, &m.vbo);

    m.vertices.clear();
    m.indices.clear();
}

void drawMesh(physics::Mesh& m) {
	if (!m.hasRenderContext) {
		prepareMeshRenderContext(m);
	}

    glBindVertexArray(m.vao);
    glDrawElements(GL_TRIANGLES, m.indices.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

} // namespace phe::graphics

