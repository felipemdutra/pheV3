#include "graphics/renderer.hpp"
#include "graphics/window.hpp"
#include "physics/rigidbody.hpp"
#include <glm/ext/quaternion_trigonometric.hpp>
#include <GLFW/glfw3.h>

#define WIDTH 800
#define HEIGHT 600

using namespace phe;
using namespace glm;

float lastTime = 0.0f;

int main() {
    auto window = graphics::createWindow(WIDTH, HEIGHT, "pheV3");
    if (!window) {
        return 1;
    }

    if (graphics::initWindow(window) < 0) {
        graphics::destroyWindow(window);
        return 1;
    }

    graphics::Renderer renderer = graphics::renderInit(WIDTH, HEIGHT);

    physics::RigidBody rb(vec3(1.0f, 1.0f, 1.0f), vec3(0.8f, 0.2f, 0.2f), 1.0f);
    physics::setPosition(rb, 0.0f, 0.0f, -5.0f);

    bool isTrue = true;
    const glm::vec3 GRAVITY(0.0f, -9.81f, 0.0f);

    while (!graphics::shouldClose(window)) {
        float currentTime = static_cast<float>(glfwGetTime());
        float dt = currentTime - lastTime;
        lastTime = currentTime;
        graphics::renderClear(0.2f, 0.3f, 0.3f, 1.0f);

        if (isTrue) {
            physics::applyForce(rb, vec3(-5.0f, 40.0f, 30.0f), vec3(0.2f, 0.2f, 0.1f));
            isTrue = false;
        }

        // Apply gravity
        physics::applyForce(rb, GRAVITY, vec3(0.0f, 0.0f, 0.0f));

        physics::integrateForces(rb, dt);
        physics::integrateVelocity(rb, dt);

        graphics::drawRigidBody(rb, renderer);

        graphics::swapBuffers(window);
        graphics::pollEvents();
    }

    graphics::destroyWindow(window);
    return 0;
}

