#include <GL/glew.h>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>

#include "graphics/renderer.hpp"

#include <graphics/mesh.hpp>
#include <physics/world.hpp>

namespace phe::graphics {

void renderClear(float r, float g, float b, float a) {
    glClearColor(r, g, b, a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

Renderer renderInit(const float windowWidth, const float windowHeight) {

	const char* fragmentShader = R"(
		#version 330 core

		in vec3 vertexColor;
		out vec4 FragColor;

		void main() {
		    FragColor = vec4(vertexColor, 1.0); // Use vertex color as fragment color
		}
	)";

	const char* vertexShader = R"(
		#version 330 core

		layout (location = 0) in vec3 aPos;   // Vertex position
		layout (location = 1) in vec3 aColor; // Vertex color

		out vec3 vertexColor; // Pass color to fragment shader

		uniform mat4 model;
		uniform mat4 view;
		uniform mat4 projection;

		void main() {
		    vertexColor = aColor;
		    gl_Position = projection * view * model * vec4(aPos, 1.0); // Transform vertex
		}
	)";

    Renderer r;
    r.shader = new Shader(vertexShader, fragmentShader);
    r.shader->use();

    glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 5.0f),
            glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    glm::mat4 projection = glm::perspective(glm::radians(60.0f),
            (float)windowWidth/(float)windowHeight, 0.1f, 1000.0f);

    r.shader->setMat4("view", view);
    r.shader->setMat4("projection", projection);

    glEnable(GL_DEPTH_TEST);

    return r;
}

void drawRigidBody(physics::RigidBody& rb, Renderer& r) {
    auto model = math::toMatrix(rb.trans);
    r.shader->setMat4("model", model);
    drawMesh(rb.mesh);
}

void drawPhysicsWorld(physics::PhysicsWorld& pw, Renderer& r) {
	for (auto& body : pw.getRigidBodies()) {
		drawRigidBody(*body, r);
	}
}

Renderer::Renderer() : shader(nullptr){ }
Renderer::~Renderer() { delete shader; }
    
} // namespace phe::graphics

