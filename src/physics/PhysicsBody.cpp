#include "physics.h"
#include <util/util.h>

PhysicsBody::PhysicsBody(std::shared_ptr<PhysicsShape> shape, double mass, PhysicsLayer layer)
:shape(shape), mass(mass), transform(glm::vec3(0.0), glm::quat(1.0, 0.0, 0.0, 0.0)), layer(layer)
{
    Ibody = shape->get_body_mat(mass);
    IbodyInv = glm::inverse(Ibody);
}

PhysicsBody::PhysicsBody(std::shared_ptr<PhysicsShape> shape, const glm::vec3& position, const glm::quat& orientation, double mass, PhysicsLayer layer)
:shape(shape), transform(position, orientation), mass(mass), layer(layer)
{
    Ibody = shape->get_body_mat(mass);
    IbodyInv = glm::inverse(Ibody);
}

const glm::vec3& PhysicsBody::get_position() const
{
    return transform.position;
}

const glm::quat& PhysicsBody::get_orientation() const
{
    return transform.orientation;
}

glm::mat4 PhysicsBody::get_world_matrix() const
{
    return glm::translate(glm::mat4(1.0), transform.position) * glm::toMat4(transform.orientation);
}

void PhysicsBody::add_force(const glm::vec3& force)
{
    this->force += force;
}

void PhysicsBody::add_torque(const glm::vec3& torque)
{
    this->torque += torque;
}

void PhysicsBody::add_impulse(const glm::vec3& impulse)
{
    this->impulse += impulse;
}

void PhysicsBody::add_torque_impulse(const glm::vec3& torque_impulse)
{
    this->torque_impulse += torque_impulse;
}

void PhysicsBody::set_linear_velocity(const glm::vec3& v)
{
    linear_momentum.x = v.x * mass;
    linear_momentum.y = v.y * mass;
    linear_momentum.z = v.z * mass;
}

void PhysicsBody::set_angular_velocity(const glm::vec3& omega)
{
    glm::mat3 rotation = glm::toMat3(glm::normalize(transform.orientation));
    glm::mat3 inertia_tensor = rotation * Ibody * glm::transpose(rotation);
    angular_momentum = inertia_tensor * omega;
}

void PhysicsBody::set_position(const glm::vec3& pos)
{
    transform.position = pos;
}

void PhysicsBody::set_orientation(const glm::vec3& orientation)
{
    transform.orientation = orientation;
}

void PhysicsBody::step(double delta)
{

    // Add the forces that were accumulated over the frame to their respective momenta
    linear_momentum += impulse;
    force *= delta;  // Force * time = impulse = change in momentum caused by force F over time t
    linear_momentum += force;

    angular_momentum += torque_impulse;
    torque *= delta;
    angular_momentum += torque;

    // Get both velocities
    glm::vec3 linear_velocity;
    linear_velocity.x = linear_momentum.x / mass;
    linear_velocity.y = linear_momentum.y / mass;
    linear_velocity.z = linear_momentum.z / mass;

    transform.orientation = glm::normalize(transform.orientation);
    glm::mat3 R = glm::toMat3(transform.orientation);
    glm::mat3 inertia_inv = R * IbodyInv * glm::transpose(R);

    glm::vec3 angular_velocity = inertia_inv * angular_momentum;
    glm::quat angular_vel_quat = glm::quat(0.0f, angular_velocity);
    glm::quat orientation_delta = angular_vel_quat * transform.orientation;
    orientation_delta *= 0.5 * delta;

    // Change in position = velocity * time
    glm::vec3 position_delta = linear_velocity;
    position_delta *= delta;

    transform.position += position_delta;
    transform.orientation = glm::normalize(orientation_delta + (0.5f * transform.orientation));

    force = glm::vec3(0.0);
    torque = glm::vec3(0.0);
    impulse = glm::vec3(0.0);
    torque_impulse = glm::vec3(0.0);
}

std::vector<uint8_t> PhysicsBody::serialize() const 
{
    NOT_IMPLEMENTED();
    return {};
}