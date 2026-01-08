#include "graphics/renderer.hpp"
#include "graphics/window.hpp"
#include "physics/rigidbody.hpp"
#include <GLFW/glfw3.h>

#define WIDTH 900 
#define HEIGHT 700 

using namespace phe;
using namespace glm;

int main() {
    auto window = graphics::createWindow(WIDTH, HEIGHT, "Single Body Test");
    if (!window) {
        return 1;
    }

    if (graphics::initWindow(window) < 0) {
        graphics::destroyWindow(window);
        return 1;
    }

    graphics::Renderer renderer = graphics::renderInit(WIDTH, HEIGHT);

    physics::RigidBody rb(vec3(1.0f, 1.0f, 1.0f), vec3(0.8f, 0.2f, 0.2f), 1.0f, true);
    physics::setPosition(rb, 0.0f, 0.0f, -5.0f);

    const glm::vec3 GRAVITY(0.0f, -9.80665f, 0.0f);

    bool isTrue = false;
    float startTime = glfwGetTime();
	float lastTime = startTime;

    while (!graphics::shouldClose(window)) {
        float currentTime = static_cast<float>(glfwGetTime());
        float dt = currentTime - lastTime;
        lastTime = currentTime;
        graphics::renderClear(0.2f, 0.3f, 0.3f, 1.0f);

        if (currentTime - startTime >= 1.5f) {
            if (!isTrue) {
                physics::applyForce(rb, vec3(100.0f, 1800.0f, -400.0f), vec3(0.050f, -0.1f, 0.0f));
                isTrue = true;
            }
            // Apply gravity
            physics::applyForce(rb, GRAVITY, vec3(0.0f, 0.0f, 0.0f));

            physics::integrateForces(rb, dt);
            physics::integrateVelocity(rb, dt);
        }

        graphics::drawRigidBody(rb, renderer);

        graphics::swapBuffers(window);
        graphics::pollEvents();
    }

    graphics::destroyWindow(window);
    return 0;
}

