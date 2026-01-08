#include <glm/glm.hpp>

#include "graphics/renderer.hpp"
#include "graphics/window.hpp"
#include "physics/world.hpp"
#include "physics/rigidbody.hpp"

using namespace phe;

int main() {
    auto window = graphics::createWindow(800, 600, "Box Stacking Test");
    if (!window) {
        return 1;
    }

    if (graphics::initWindow(window) < 0) {
        return 1;
    }

    auto renderer = graphics::renderInit(window->width, window->height);

    physics::RigidBody rb1(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, true);
    physics::RigidBody rb2(glm::vec3(10.0f, 1.0f, 10.0f), glm::vec3(0.0f, 0.0f, 1.0f), 1.0f, false);

    physics::setPosition(rb1 ,glm::vec3(0.0f, 3.0f, -4.0f));
    physics::setPosition(rb2, glm::vec3(0.0f, -3.0f, -4.0f));

    physics::PhysicsWorld world;

    world.addRigidBody(&rb2);
    world.addRigidBody(&rb1);

    physics::applyForce(rb1, glm::vec3(0.0f, -6.0f, 0.0f), glm::vec3(0.1f, 0.0f, 0.1f));
	float lastTime = static_cast<float>(glfwGetTime()) - 0.1;

    while (!graphics::shouldClose(window)) {
        graphics::renderClear(0.2f, 0.3f, 0.3f, 1.0f);

        float currentTime = static_cast<float>(glfwGetTime());
        float dt = currentTime - lastTime;
        world.update(dt);
    	graphics::drawPhysicsWorld(world, renderer);

        lastTime = currentTime;

        graphics::pollEvents();
        graphics::swapBuffers(window);
    }

    graphics::destroyWindow(window);
}


