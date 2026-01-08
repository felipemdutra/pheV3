#include <cstddef>

#include "physics/world.hpp"
#include "physics/collision.hpp"
#include "physics/utils.hpp"
#include "physics/rigidbody.hpp"

namespace phe::physics {

PhysicsWorld::PhysicsWorld() {
    gp.gridSize = 10.0f;
    gp.gridCells.clear();
}

void PhysicsWorld::update(float dt) {

	// FIXME this is broken
    // auto filteredBodies = collision::broadPhaseFilter(bodies, gp);
    //
    // for (size_t i = 0; i < filteredBodies.size(); i++) {
    //     for (size_t j = i+1; j < filteredBodies.size(); j++) {
    //         auto& b1 = filteredBodies[i];
    //         auto& b2 = filteredBodies[j];
    //         auto info = collision::areColliding(*b1, *b2);
    //         if (info.isColliding) {
    //             collision::resolveCollision(*b1, *b2, info);
    //         }
    //     }
    // }

	for (int i = 0; i < bodies.size(); i++) {
		RigidBody* a = bodies[i];

		if (a->isDynamic) {
			applyForce(*a, GRAVITY, glm::vec3(0.0f, 0.0f, 0.0f));
			integrateForces(*a, dt);
			integrateVelocity(*a, dt);
		}

		for (int j = 0; j < bodies.size(); j++) {
			if (j == i) {
				continue;
			}

			RigidBody* b = bodies[j];

			auto info = collision::areColliding(*a, *b);

			if (info.isColliding) {
			    collision::resolveCollision(*a, *b, info);
			}
		}
	}
}

void PhysicsWorld::addRigidBody(RigidBody* rb) {
    bodies.push_back(rb);
}

const std::vector<RigidBody*>& PhysicsWorld::getRigidBodies() const {
	return bodies;
}

}

