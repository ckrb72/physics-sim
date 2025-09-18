#pragma once
#include <vector>
#include <queue>
#include <memory>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>


struct AABBox
{
    glm::vec3 half_extents;
    glm::vec3 position;
};

class PhysicsShape
{
    public:
        virtual bool collide() = 0;
        virtual glm::mat3 get_body_mat() = 0;
        virtual AABBox get_aabb() = 0;
};

//Maybe make a distinction between collision shapes and physics shapes???
//Physics shapes would have inertia tensor while collision shape would just have collision detection stuff

class BoxShape : public PhysicsShape
{
    private:
        glm::vec3 half_extent;

    public:
        BoxShape(glm::vec3 half_extent);
        bool collide() override;
        glm::mat3 get_body_mat() override;
        AABBox get_aabb() override;
};

// The origin of this sphere is in "body space" so it will always be 0, 0, 0. In the physics body it is attached to the position will change
class SphereShape : public PhysicsShape
{
    private:
        double r;

    public:
        SphereShape(double radius);
        bool collide() override;
        glm::mat3 get_body_mat() override;
        AABBox get_aabb() override;
};

// The point used for this plane is in "body space" so it will always be 0, 0, 0. In the physics body it is attached to the position will change
class PlaneShape : public PhysicsShape
{
    private:
        glm::vec3 norm;
        glm::vec3 extent;

    public:
        PlaneShape(const glm::vec3& norm, const glm::vec3& extent);
        bool collide() override;
        glm::mat3 get_body_mat() override;
        AABBox get_aabb() override;
};

/*class ConeShape : public PhysicsShape
{
    private:
        double height;
        double radius;

    public:
        ConeShape(double radius, double height);
        bool collide() override;
        glm::mat4 get_body_mat() override;
};

class CylinderShape : public PhysicsShape
{
    private:
        double radius;
        double height;
    
    public:
        CylinderShape(double radius, double height);
        bool collide() override;
        glm::mat4 get_body_mat() override;
};*/


class PhysicsBody
{
    private:
        double mass;
        glm::mat4 Ibody;
        glm::mat4 IbodyInv;
        std::shared_ptr<PhysicsShape> shape;

        glm::vec3 position;
        glm::quat orientation;

        glm::vec3 linear_momentum;
        glm::vec3 angular_momentum;

        glm::vec3 force;
        glm::vec3 torque;

        struct BodyDifferentials
        {
            glm::mat3 Iinv;
            glm::vec3 v;
            glm::vec3 omega;
            glm::mat3 R;
            glm::quat qdot;
        };

        BodyDifferentials ddt();

    public:
        PhysicsBody(std::shared_ptr<PhysicsShape> shape, double mass);
        PhysicsBody(std::shared_ptr<PhysicsShape> shape, const glm::vec3& position, const glm::quat& orientation, double mass);

        const glm::vec3& get_position() const;
        const glm::quat& get_orientation() const;
        glm::mat4 get_world_matrix() const;

        void set_linear_velocity(const glm::vec3& v);
        void set_angular_velocity(const glm::vec3& omega);
        void set_position(const glm::vec3& pos);
        void set_orientation(const glm::vec3& orientation);

        void add_force(const glm::vec3& force);
        void add_torque(const glm::vec3& torque);

        void step(double delta);
};

class PhysicsWorld
{
    private:
        std::vector<PhysicsBody> bodies;

    public:
        PhysicsWorld();
        int32_t create_body(std::shared_ptr<PhysicsShape> shape, double mass);
        int32_t create_body(std::shared_ptr<PhysicsShape> shape, const glm::vec3& position, const glm::quat& orientation, double mass);
        void remove(int32_t id);

        // Body manipulation functions
        void set_linear_velocity(int32_t id, const glm::vec3& v);
        void set_angular_velocity(int32_t id, const glm::vec3& omega);
        glm::mat4 get_world_matrix(int32_t id);

        // TODO: Updates with 1 / 60 second granularity. If delta > 1 / 60 the integration step is done multiple times
        void update(double delta);  // This is where the integration actually occurs
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
/*class PhysicsEngine
{
    private:
        std::vector<RigidBody> bodies;

    public:
        PhysicsEngine(uint32_t buf_size);

        int32_t add();

        void remove();

        std::queue<BodyUpdate> update(double delta);
};*/