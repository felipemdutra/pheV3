#include <cstdio>
#include <glm/glm.hpp>

#include "physics/world.hpp"
#include "physics/rigidbody.hpp"

using namespace phe;

int main() {

	physics::RigidBody rb1(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, true);
	physics::RigidBody rb2(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), 1.0f, true);

	physics::setPosition(rb1, glm::vec3(-4.0f, 1.0f, 0.0f));
	physics::setPosition(rb2, glm::vec3(4.0f, 1.0f, 0.0f));

	physics::setRotation(rb2, 45.0f, glm::vec3(0.3f, 0.7f, 0.5f));

	physics::applyForce(rb1, glm::vec3(70.0f, 60.0f, -80.0f), glm::vec3(0.1, 0.1, 0.0f));
	physics::applyForce(rb2, glm::vec3(-70.0f, 60.0f, -80.0f), glm::vec3(0.0, 0.1, 0.0));

	physics::PhysicsWorld world;

	world.addRigidBody(&rb2);
	world.addRigidBody(&rb1);

	int steps = 0;
	bool run = true;

	rb1.on_collision = [&] (physics::RigidBody& rb) {
		run = false;
	};

    while (run) {
        world.update(0.02); // 20ms step

    	steps ++;
    }

	printf("Collision after %d steps\n", steps);
}


