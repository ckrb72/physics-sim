#pragma once
#include <vector>
#include <queue>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

struct RigidBody
{
    double mass;
    glm::mat3 Ibody;
    glm::mat3 IbodyInv;

    glm::vec3 x;    // pos
    glm::quat q;    // orientation expressed as a quaternion
    glm::vec3 P;    // linear momentum
    glm::vec3 L;    // angular momentum

    glm::mat3 Iinv; // Inverse inertia tensor
    glm::vec3 v;    // velocity
    glm::vec3 omega;// angular velocity
    glm::mat3 R;    // Orientation derived from quaternion q
    glm::quat qdot; // Change of orientation quaternion

    glm::vec3 force;
    glm::vec3 torque;
};

struct Impulse
{
    uint32_t rb_id;     // Id of the rigid body
    double time;
    glm::vec3 pos;      // position in body space 
    glm::vec3 force;    // force vector
};

struct BodyUpdate
{
    uint32_t rb_id;
    glm::vec3 pos;
    glm::quat orientation;
};

// Idea for Engine Architecture
class PhysicsEngine
{
    private:
        std::vector<RigidBody> bodies;

    public:
        PhysicsEngine(uint32_t buf_size);

        int32_t add();

        void remove();

        std::queue<BodyUpdate> update(double delta);
};