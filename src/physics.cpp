#include "physics.h"
#include "util.h"

PhysicsBody::PhysicsBody(std::shared_ptr<PhysicsShape> shape, double mass)
:shape(shape), mass(mass), position(0.0), orientation(glm::angleAxis(0.0f, glm::vec3(1.0, 0.0, 0.0)))
{
    Ibody = shape->get_body_mat();

    // Shape's body matrix isn't scaled by mass so need to do that here
    Ibody[0][0] *= mass;
    Ibody[1][1] *= mass;
    Ibody[2][2] *= mass;

    IbodyInv = glm::inverse(Ibody);
}

PhysicsBody::PhysicsBody(std::shared_ptr<PhysicsShape> shape, const glm::vec3& position, const glm::quat& orientation, double mass)
:shape(shape), position(position), orientation(orientation), mass(mass)
{
    Ibody = shape->get_body_mat();

    // Shape's body matrix isn't scaled by mass so need to do that here
    Ibody[0][0] *= mass;
    Ibody[1][1] *= mass;
    Ibody[2][2] *= mass;

    IbodyInv = glm::inverse(Ibody);
}

const glm::vec3& PhysicsBody::get_position() const
{
    return position;
}

const glm::quat& PhysicsBody::get_orientation() const
{
    return orientation;
}

glm::mat4 PhysicsBody::get_world_matrix() const
{
    return glm::translate(glm::mat4(1.0), position) * glm::toMat4(orientation);
}

void PhysicsBody::add_force(const glm::vec3& force)
{
    this->force += force;
}

void PhysicsBody::add_torque(const glm::vec3& torque)
{
    this->torque += torque;
}

void PhysicsBody::step(double delta)
{
    
}

BoxShape::BoxShape(glm::vec3 half_extent)
:half_extent(half_extent)
{}

bool BoxShape::collide()
{
    NOT_IMPLEMENTED();
    return false;
}

glm::mat3 BoxShape::get_body_mat()
{
    glm::vec3 dimensions;
    dimensions.x = half_extent.x * 2.0;
    dimensions.y = half_extent.y * 2.0;
    dimensions.z = half_extent.z * 2.0;

    return {
        {1.0 / 12.0 * ( (dimensions[1] * dimensions[1]) + (dimensions[2] * dimensions[2]) ), 0.0, 0.0},
        {0.0, 1.0 / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[2] * dimensions[2]) ), 0.0},
        {0.0, 0.0, 1.0 / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[1] * dimensions[1]) )}
    };
}

AABBox BoxShape::get_aabb()
{
    NOT_IMPLEMENTED();
    return AABBox{};
}

SphereShape::SphereShape(double radius)
:r(radius)
{}

bool SphereShape::collide()
{
    NOT_IMPLEMENTED();
    return false;
}

glm::mat3 SphereShape::get_body_mat()
{
    NOT_IMPLEMENTED();
    return glm::mat4(1.0f);
}

AABBox SphereShape::get_aabb()
{
    NOT_IMPLEMENTED();
    return AABBox{};
}

PlaneShape::PlaneShape(const glm::vec3& norm, const glm::vec3& extent)
:norm(norm), extent(extent)
{}

bool PlaneShape::collide()
{
    NOT_IMPLEMENTED();
    return false;
}

glm::mat3 PlaneShape::get_body_mat()
{
    NOT_IMPLEMENTED();
    return glm::mat4(1.0f);
}

AABBox PlaneShape::get_aabb()
{
    NOT_IMPLEMENTED();
    return AABBox{};
}