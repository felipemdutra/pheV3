#ifndef PHYSICS_WORLD_HPP
#define PHYSICS_WORLD_HPP

#include <vector>

#include "graphics/renderer.hpp"
#include "physics/rigidbody.hpp"

namespace phe::physics {

/*
 * PhysicsWorld class is ONLY responsible for applying GRAVITY, integrating
 * forces to velocity, and integrating velocity to position of all rigid bodies
 * it owns
 */
class PhysicsWorld {
private:
    std::vector<RigidBody*> bodies;
    const glm::vec3 GRAVITY = glm::vec3(0.0f, -9.80665, 0.0f);

public:
    void update(float dt, graphics::Renderer& r);

    void addRigidBody(RigidBody* rb);
};

}
#endif

