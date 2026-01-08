#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "physics/collision.hpp"
#include "physics/utils.hpp"
#include "physics/rigidbody.hpp"

namespace phe::physics::collision {

static std::tuple<int, int, int> findBodyGridCell(RigidBody* b, GridPartition& gp) {
    int gridX = (int) std::floor(b->trans.translation.x / gp.gridSize);
    int gridY = (int) std::floor(b->trans.translation.y / gp.gridSize);
    int gridZ = (int) std::floor(b->trans.translation.z / gp.gridSize);

    return std::make_tuple(gridX, gridY, gridZ);
}

/**
 * @param cell The cell you want to check the neighbors of.
 * @param out The output vector that is going to have all the rigid bodies of the neighboring cells
 */
static void checkNeighboringCells(std::tuple<int, int, int> cell, RigidBody* currBody, GridPartition& gp, std::vector<RigidBody*>& out) {
    std::unordered_set<RigidBody*> addedBodies;

    // Go over every neighbor of the current cell in a 3x3x3 grid
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            for (int z = -1; z <= 1; ++z) {
                
                // Skip our current cell.
                if (x == 0 && y == 0 && z == 0) continue;

                // Get the neighbor cell by adding current cell coordinates + current neighbor coordinates (represented by x, y and z)
                auto neighborCell = glm::ivec3(std::get<0>(cell) + x, std::get<1>(cell) + y, std::get<2>(cell) + z);

                // O(1) look up time
                auto it = gp.gridCells.find(neighborCell);
                if (it == gp.gridCells.end()) {
                    continue;
                }

                // Now add the bodies from neighbor cell to the output, ONLY IF
                // it's not found in addedBodies.
                for (RigidBody* neighborB : it->second) {
                    if (neighborB == currBody) continue;

                    auto itt = addedBodies.find(neighborB);
                    // If we DIDN'T find neighbor body in the addedBodies set,
                    // add it to the output.
                    if (itt == addedBodies.end()) {
                        out.push_back(neighborB);
                        addedBodies.insert(neighborB);
                    }
                }
            }
        }
    }
}

void resolveCollision(RigidBody &a, RigidBody &b, CollisionInfo collisionInfo) {
    glm::vec3 mpa = collisionInfo.mpa;
    float penetrationDepth = collisionInfo.penetrationDepth;

    glm::vec3 contactPointA = findContactPoint(a, mpa, penetrationDepth);
    glm::vec3 contactPointB = findContactPoint(b, -mpa, penetrationDepth);

    glm::vec3 rA = contactPointA - a.trans.translation;
    glm::vec3 rB = contactPointB - b.trans.translation;

    // Velocity of rigid body A
    glm::vec3 vA = a.linearVel + (glm::cross(a.angularVel, rA));

    // Velocity of rigid body B
    glm::vec3 vB = b.linearVel + (glm::cross(b.angularVel, rB));

    // We use relative velocity of both bodies to find the impulse needed
    // to push them apart. It's the reaction of the collision.
    glm::vec3 relativeVel = vA - vB;
	float velocityNormal = glm::dot(relativeVel, mpa);

	if (velocityNormal > 0.0f) {
		return;
	}

	float invMassA = a.isDynamic ? 1.0f / a.mass : 0.0f;
	float invMassB = b.isDynamic ? 1.0f / b.mass : 0.0f;

	const float percent = 0.8f;	// correction strength
	const float slop = 0.01f;

	float correctionMag = glm::max(penetrationDepth - slop, 0.0f) / (invMassA + invMassB) * percent;
	glm::vec3 correction = correctionMag * mpa;

	if (a.isDynamic) a.trans.translation += correction * invMassA;
	if (b.isDynamic) b.trans.translation -= correction * invMassB;

	glm::mat3 invInertiaA = a.isDynamic ? glm::inverse(a.inertiaTensor) : glm::mat3(0.0f);
	glm::mat3 invInertiaB = b.isDynamic ? glm::inverse(b.inertiaTensor) : glm::mat3(0.0f);

    float j = -(1 + RESTITUTION_CO) * velocityNormal /
        (
          invMassA +
          invMassB +
          glm::dot(glm::cross(rA, mpa), invInertiaA * glm::cross(rA, mpa)) +
          glm::dot(glm::cross(rB, mpa), invInertiaB * glm::cross(rB, mpa))
        );

    // Impulse is given by j * n (Minimum Penetration Axis)
    glm::vec3 impulse = j * mpa;

	// notify handlers of the collision
	a.on_collision(b);
	b.on_collision(a);

    applyImpulse(a, impulse, rA);
    applyImpulse(b, -impulse, rB);
}

std::vector<RigidBody*> broadPhaseFilter(const std::vector<RigidBody*>& bodies, GridPartition& gp) {
    std::vector<RigidBody*> possibleCollisions;

    for (RigidBody* b : bodies) {
        std::tuple<int, int, int> cell = findBodyGridCell(b, gp);

        checkNeighboringCells(cell, b, gp, possibleCollisions);
    }

    return possibleCollisions;
}

} // namespace phe::physics::collision

