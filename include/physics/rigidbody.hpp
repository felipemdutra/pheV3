#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

#include <functional>
#include <glm/fwd.hpp>
#include <glm/glm.hpp>

#include "mesh.hpp"
#include "transform.hpp"

namespace phe::physics {

struct RigidBody {
    math::Transform trans = math::Transform();

    glm::vec3 linearVel = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 linearAcc = glm::vec3(0.0f, 0.0f, 0.0f);

    glm::vec3 angularVel = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 angularAcc = glm::vec3(0.0f, 0.0f, 0.0f);

    glm::vec3 size = glm::vec3(1.0f, 1.0f, 1.0f);

    glm::mat3 inertiaTensor = glm::mat4(1.0f);

    float mass = 1.0f;
    bool isDynamic = false;

    Mesh mesh;

    bool operator!=(const RigidBody& other) const {
        return !(this->trans == other.trans);
    }

	std::function<void(RigidBody&)> on_collision = [] (RigidBody& other) {};

    RigidBody(glm::vec3 size, glm::vec3 rgb, float mass, bool isDynamic);
    ~RigidBody();
};

/**
 * Applies linear motion to the rigid body, by updating its translation based
 * on the applied force translation vector. The force does not affect the 
 * angular velocity of the Rigid Body, only it's translation.
 *
 * @param rb The Rigid Body to which the force is going to be applied to.
 * @param f The force applied to the Rigid Body.
 */
void applyLinearForce(RigidBody& rb, glm::vec3 f);

/**
 * Applies rotational motion to the Rigid Body by updating its rotation based
 * on the applied torque. The force does not affect the linear velocity of the
 * Rigid Body, only its orientation.
 *
 * @param rb The Rigid Body to which the torque is applied.
 * @param f The force applied to the Rigid Body, generating torque.
 * @param r The vector representing the distance from the Rigid Body's center 
 * of mass to the point where the force was applied.
 */
void applyAngularForce(RigidBody& rb, glm::vec3 f, glm::vec3 r);

void applyLinearImpulse(RigidBody& rb, glm::vec3 i);

void applyAngularImpulse(RigidBody& rb, glm::vec3 i, glm::vec3 r);
/**
 * Applies both angular and linear forces
 *
 * @param rb The Rigid Body to which the force and torque is applied.
 * @param f The force applied to the Rigid Body. Also used to generate the torque.
 * @param r The vector representing the diestnace from the Rigid Body's center
 * of mass to the point where the force was applied.
 */
void applyForce(RigidBody& rb, glm::vec3 f, glm::vec3 r);

void applyImpulse(RigidBody& rb, glm::vec3 i, glm::vec3 r);

/**
 * Integrates both linear and angular acceleration to linear velocity and 
 * angular velocity respectively.
 *
 * @param rb The Rigid Body to which its acceleration is going to be integrated
 * into its velocity
 * @param dt Delta time.
 */
void integrateForces(RigidBody& rb, float dt);
void integrateVelocity(RigidBody& rb, float dt);

void setPosition(RigidBody& rb, float x, float y, float z);
void setPosition(RigidBody& rb, glm::vec3 newPosition);

void setRotation(RigidBody& rb, glm::quat q);
void setRotation(RigidBody &rb, float angleDegrees, glm::vec3 axis);

}

#endif

