#include "physics.h"

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

void PhysicsBody::set_linear_velocity(const glm::vec3& v)
{
    linear_momentum.x = v.x * mass;
    linear_momentum.y = v.y * mass;
    linear_momentum.z = v.z * mass;
}

void PhysicsBody::set_position(const glm::vec3& pos)
{
    position = pos;
}

void PhysicsBody::set_orientation(const glm::vec3& orientation)
{
    this->orientation = orientation;
}

void PhysicsBody::step(double delta)
{
    glm::vec3 velocity;
    velocity.x = linear_momentum.x / mass;
    velocity.y = linear_momentum.y / mass;
    velocity.z = linear_momentum.z / mass;
    velocity *= delta;

    position += velocity;

    force = glm::vec3(0.0);
    torque = glm::vec3(0.0);
}