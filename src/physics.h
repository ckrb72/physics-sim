#pragma once
#include <vector>
#include <queue>
#include <memory>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

class CollisionShape
{
    public:
        virtual bool collide() = 0;
        virtual glm::mat4 body_mat() = 0;
};
/*
class BoxShape : public CollisionShape
{
    private:
        glm::vec3 half_extent;

    public:
        BoxShape(glm::vec3 half_extent);
        bool collide() override;
        glm::mat4 body_mat() override;
};

// The origin of this sphere is in "body space" so it will always be 0, 0, 0. In the physics body it is attached to the position will change
class SphereShape : public CollisionShape
{
    private:
        double r;

    public:
        SphereShape(double radius);
        bool collide() override;
        glm::mat4 body_mat() override;
};

// The point used for this plane is in "body space" so it will always be 0, 0, 0. In the physics body it is attached to the position will change
class PlaneShape : public CollisionShape
{
    private:
        glm::vec3 norm;

    public:
        PlaneShape(glm::vec3 norm);
        bool collide() override;
        glm::mat4 body_mat() override;
};*/

class PhysicsBody
{
    private:
        double mass;
        glm::mat4 Ibody;
        glm::mat4 IbodyInv;
        std::shared_ptr<CollisionShape> collider;

        glm::vec3 position;
        glm::quat orientation;

        glm::vec3 linear_momentum;
        glm::vec3 angular_momentum;

    public:
        PhysicsBody(std::shared_ptr<CollisionShape> collider, double mass);

        glm::vec3 get_position() const;
        glm::quat get_orientation() const;
        glm::mat4 get_world_matrix() const;
};

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