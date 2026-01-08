#ifndef RENDERER_HPP
#define RENDERER_HPP

#include "graphics/shader.hpp"
#include "physics/rigidbody.hpp"

namespace phe::physics {
	class PhysicsWorld;
}
namespace phe::graphics {
    struct Renderer {
        Shader* shader = nullptr;

        Renderer();
        ~Renderer();
    };

    Renderer renderInit(const float windowWidth, const float windowHeight);

    void renderClear(float r, float g, float b, float a);

    /**
     * Draws the rigid body in world space, feeding its transformed model
     * matrix to the shader program.
     */
    void drawRigidBody(physics::RigidBody& rb, Renderer& r);

    /**
     * Draw all rigid bodies in given physics world using the
     * supplied renderer.
     */
    void drawPhysicsWorld(physics::PhysicsWorld& pw, Renderer& r);

}

#endif

