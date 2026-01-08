// This was made to test collision detection between two Rigid Bodies.
#include <glm/glm.hpp>
#include <physics/world.hpp>

#include "graphics/renderer.hpp"
#include "graphics/window.hpp"
#include "physics/collision.hpp"
#include "physics/rigidbody.hpp"

using namespace phe;

const glm::vec3 GRAVITY(0.0f, -9.81, 0.0f);

int main() {
    auto window = graphics::createWindow(800, 600, "Collision Detection Test");
    if (!window) {
        return 1;
    }

    if (graphics::initWindow(window) < 0) {
        return 1;
    }

    auto renderer = graphics::renderInit(window->width, window->height);

    physics::RigidBody rb1(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, true);
    physics::RigidBody rb2(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), 1.0f, true);

    physics::setPosition(rb1, glm::vec3(-4.0f, 1.0f, 0.0f));
    physics::setPosition(rb2, glm::vec3(4.0f, 1.0f, 0.0f));

    physics::setRotation(rb2, 45.0f, glm::vec3(0.3f, 0.7f, 0.5f));

    physics::applyForce(rb1, glm::vec3(70.0f, 60.0f, -80.0f), glm::vec3(0.1, 0.1, 0.0f));
    physics::applyForce(rb2, glm::vec3(-70.0f, 60.0f, -80.0f), glm::vec3(0.0, 0.1, 0.0));

	physics::PhysicsWorld world;

	world.addRigidBody(&rb2);
	world.addRigidBody(&rb1);

	float lastTime = static_cast<float>(glfwGetTime()) - 0.1;

	rb1.on_collision = [&] (physics::RigidBody& rb) {
		printf("Collision!\n");
	};

    while (!graphics::shouldClose(window)) {
        float currentTime = static_cast<float>(glfwGetTime());
        float dt = currentTime - lastTime;
        lastTime = currentTime;

        graphics::renderClear(0.2f, 0.3f, 0.3f, 1.0f);
        
        world.update(dt);

        graphics::drawRigidBody(rb1, renderer);
        graphics::drawRigidBody(rb2, renderer);

        graphics::pollEvents();
        graphics::swapBuffers(window);
    }

    graphics::destroyWindow(window);
}

