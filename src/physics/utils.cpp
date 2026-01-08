#include "physics/utils.hpp"
#include "physics/mesh.hpp"
#include "physics/rigidbody.hpp"
#include <algorithm>
#include <cfloat>
#include <glm/geometric.hpp>

namespace phe::physics::collision {

float projectOntoAxis(const glm::vec3& vertex, const glm::vec3& axis) {
    return glm::dot(vertex, axis);
}

bool projectionsOverlap(float minA, float maxA, float minB, float maxB) {
    return !(maxA < minB || maxB < minA);
}
    
std::vector<glm::vec3> computeCubeNormals(const RigidBody &rb) {
    static const std::vector<glm::vec3> cubeLocalNormals = {
        // Normals for each face of the cube
        { 1,  0,  0},
        {-1,  0,  0},
        { 0,  1,  0},
        { 0, -1,  0},
        { 0,  0,  1},
        { 0,  0, -1}
    };

    std::vector<glm::vec3> worldNormals;
    worldNormals.reserve(cubeLocalNormals.size());

    for (const auto& normal : cubeLocalNormals) {
        // Get world normals by transforming local normals with the rigid body's rotation
        worldNormals.push_back(rb.trans.rotation * normal);
    }

    return worldNormals;
}

std::vector<glm::vec3> computeCubeEdges(const RigidBody& rb) {
    static const std::vector<glm::vec3> cubeLocalEdges = {
        {1, 0, 0}, // X axis
        {0, 1, 0}, // Y axis
        {0, 0, 1}  // Z axis
    };

    std::vector<glm::vec3> worldEgdes;
    worldEgdes.reserve(cubeLocalEdges.size());

    for (const auto& edge : cubeLocalEdges) {
        worldEgdes.push_back(rb.trans.rotation * edge);
    }

    return worldEgdes;
}

CollisionInfo areColliding(RigidBody& a, RigidBody& b) {
    std::vector<glm::vec3> axes;    
    CollisionInfo collisionInfo {};

	if (!a.isDynamic && !b.isDynamic) {
		return collisionInfo;
	}

    auto normalsA = computeCubeNormals(a);
    auto normalsB = computeCubeNormals(b);

    axes.insert(axes.begin(), normalsA.begin(), normalsA.end());
    axes.insert(axes.begin(), normalsB.begin(), normalsB.end());

    auto edgesA = computeCubeEdges(a);
    auto edgesB = computeCubeEdges(b);

    // Calculate axis from cross product between every edge from body A and every edge from body B
    for (auto& a : edgesA) {
        for (auto& b : edgesB) {
            glm::vec3 res = glm::normalize(glm::cross(a, b));
            // ignore 0 length cross
            if (glm::length(res) > 0.0f) {
                axes.push_back(res);
            }
        }
    }

    float lastPenetrationDepth = FLT_MAX;

    for (auto& axis : axes) {
        // Find minA, maxA, minB and maxB to check if their projection overlaps
        float minA = FLT_MAX, maxA = -FLT_MAX;
        for (int i = 0; i < physics::getNumVerts(a.mesh); ++i) {
            glm::vec3 vA = physics::getVertex(a.mesh, a.trans, i);
            float projectionA = projectOntoAxis(vA, axis);
            minA = std::min(minA, projectionA);
            maxA = std::max(maxA, projectionA);
        }

        float minB = FLT_MAX, maxB = -FLT_MAX;
        for (int i = 0; i < physics::getNumVerts(b.mesh); ++i) {
            glm::vec3 vA = physics::getVertex(b.mesh, b.trans, i);
            float projectionB = projectOntoAxis(vA, axis);
            minB = std::min(minB, projectionB);
            maxB = std::max(maxB, projectionB);
        }

        if (!projectionsOverlap(minA, maxA, minB, maxB)) {
            collisionInfo.isColliding = false;
            collisionInfo.mpa = glm::vec3(0.0f, 0.0f, 0.0f);
            collisionInfo.penetrationDepth = 0.0f;
            return collisionInfo;
        }

        float currPenetrationDepth = std::min(maxA, maxB) - std::max(minA, minB);
        if (currPenetrationDepth < lastPenetrationDepth) {
            lastPenetrationDepth = currPenetrationDepth;
            collisionInfo.mpa = axis;
        }

    }

    collisionInfo.penetrationDepth = lastPenetrationDepth;
    collisionInfo.isColliding = true;
    return collisionInfo;
}

glm::vec3 findContactPoint(RigidBody& rb, const glm::vec3& mpa, float penetrationDepth) {
    return rb.trans.translation + mpa * penetrationDepth / 2.0f;

}

} // namespace phe::physics::collision

