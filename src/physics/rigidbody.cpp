#include "physics/rigidbody.hpp"
#include <glm/ext/quaternion_geometric.hpp>
#include <glm/ext/quaternion_trigonometric.hpp>
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>

#define LINEAR_DAMPING 0.9995
#define ANGULAR_DAMPING 0.999 

static glm::mat3 calculateInertiaTensor(float mass, glm::vec3 size) {
    float w = size.x;
    float h = size.y;
    float d = size.z;

    float ix = (1.0f / 12.0f) * mass * (h*h + d*d);
    float iy = (1.0f / 12.0f) * mass * (w*w + d*d);
    float iz = (1.0f / 12.0f) * mass * (w*w + h*h);

    return glm::mat3(
        ix, 0.0f, 0.0f,
        0.0f, iy, 0.0f,
        0.0f, 0.0f, iz
    );
}

namespace phe::physics {

    RigidBody::RigidBody(glm::vec3 size, glm::vec3 rgb, float mass, bool isDynamic) 
         : size(size), mass(mass), isDynamic(isDynamic) {

    mesh = physics::createCubeMesh(size, rgb);
    inertiaTensor = calculateInertiaTensor(mass, size);
}

RigidBody::~RigidBody() {
    //graphics::destroyMesh(mesh);
}

void applyLinearForce(RigidBody &rb, glm::vec3 f) {
    rb.linearAcc += f / rb.mass; 
}

void applyAngularForce(RigidBody& rb, glm::vec3 f, glm::vec3 r) {
    glm::vec3 t = glm::cross(r, f);
    rb.angularAcc += glm::inverse(rb.inertiaTensor) * t; 
}

void applyLinearImpulse(RigidBody& rb, glm::vec3 i) {
    rb.linearVel += i / rb.mass;
}

void applyAngularImpulse(RigidBody& rb, glm::vec3 i, glm::vec3 r) {
    glm::vec3 t = glm::cross(r, i);
    rb.angularVel += glm::inverse(rb.inertiaTensor) * t;
}

void applyForce(RigidBody& rb, glm::vec3 f, glm::vec3 r) {
    applyLinearForce(rb, f);
    applyAngularForce(rb, f, r);
}

void applyImpulse(RigidBody &rb, glm::vec3 i, glm::vec3 r) {
    if (!rb.isDynamic) return;
    applyLinearImpulse(rb, i);
    applyAngularImpulse(rb, i, r);
}

void integrateForces(RigidBody& rb, float dt) {
    // Integrate linear velocity.
    rb.linearVel += rb.linearAcc * dt;
    
    // Integrate angular velocity
    rb.angularVel += rb.angularAcc * dt;

    // Reset linear and angular acceleration.
    rb.linearAcc = glm::vec3(0.0f, 0.0f, 0.0f);
    rb.angularAcc = glm::vec3(0.0f, 0.0f, 0.0f);
}

void integrateVelocity(RigidBody& rb, float dt) {
    rb.trans.translation += rb.linearVel * dt;

    glm::quat velQuat(0.0f, rb.angularVel.x, rb.angularVel.y, rb.angularVel.z);
    glm::quat dq = 0.5f * rb.trans.rotation * velQuat * dt;

    rb.trans.rotation += dq; 
    rb.trans.rotation = glm::normalize(rb.trans.rotation);

    rb.linearVel *= LINEAR_DAMPING;
    rb.angularVel *= ANGULAR_DAMPING;
}

void setPosition(RigidBody& rb, float x, float y, float z) {
    rb.trans.translation.x = x;
    rb.trans.translation.y = y;
    rb.trans.translation.z = z;
}

void setPosition(RigidBody& rb, glm::vec3 newPosition) {
    rb.trans.translation = newPosition; 
}

void setRotation(RigidBody& rb, glm::quat q) {
    rb.trans.rotation = q;
}

void setRotation(RigidBody &rb, float angleDegrees, glm::vec3 axis) {
    // First normalize the axis;
    axis = glm::normalize(axis);

    rb.trans.rotation = glm::angleAxis(glm::radians(angleDegrees), axis);
}

} // namespace phe::physics

