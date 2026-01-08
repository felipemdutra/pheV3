#ifndef COLLISION_UTILS_HPP
#define COLLISION_UTILS_HPP

#include <glm/glm.hpp>

#include "physics/rigidbody.hpp"

namespace phe::physics::collision {

struct CollisionInfo {
    bool isColliding = false;
    glm::vec3 mpa;          // Minimum Penetration Axis
    float penetrationDepth; // How deep a rigid body is inside another
};

/**
 * The parameter rb NEEDS to have a CUBE mesh.
 *
 * @return vector of face normals for a cube.
 */
std::vector<glm::vec3> computeCubeNormals(const RigidBody& rb);
std::vector<glm::vec3> computeCubeEdges(const RigidBody& rb);

CollisionInfo areColliding(RigidBody& a, RigidBody& b);

float projectOntoAxis(const glm::vec3& vertex, const glm::vec3& axis);
glm::vec3 findContactPoint(RigidBody& rb, const glm::vec3& mpa, float penetrationDepth);

} // namespace phe::physics::collision
  
#endif

