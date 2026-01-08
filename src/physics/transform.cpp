#include "../../include/physics/transform.hpp"
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

glm::mat4 phe::math::toMatrix(const Transform& t) {
    glm::mat4 translation = glm::translate(glm::mat4(1.0f), t.translation);
    glm::mat4 rotation    = glm::mat4_cast(t.rotation);
    glm::mat4 scale       = glm::scale(glm::mat4(1.0f), t.scale);

    return translation * rotation * scale;
}

