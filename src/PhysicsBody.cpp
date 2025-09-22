#include "physics.h"
#include "util.h"

PhysicsBody::PhysicsBody(std::shared_ptr<PhysicsShape> shape, double mass, PhysicsLayer layer)
:shape(shape), mass(mass), position(0.0), orientation(glm::angleAxis(0.0f, glm::vec3(1.0, 0.0, 0.0))), layer(layer)
{
    Ibody = shape->get_body_mat();

    // Shape's body matrix isn't scaled by mass so need to do that here
    Ibody[0][0] *= mass;
    Ibody[1][1] *= mass;
    Ibody[2][2] *= mass;

    IbodyInv = glm::inverse(Ibody);
}

PhysicsBody::PhysicsBody(std::shared_ptr<PhysicsShape> shape, const glm::vec3& position, const glm::quat& orientation, double mass, PhysicsLayer layer)
:shape(shape), position(position), orientation(orientation), mass(mass), layer(layer)
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

    // Add the forces that were accumulated over the frame to their respective momenta
    // TODO: Might want to multiply these times delta here?
    linear_momentum += impulse;
    force *= delta * mass;  // Force * time * mass = velocity * mass = Impulse of force F over time T on object with mass M
    linear_momentum += force;

    angular_momentum += torque_impulse;
    
    // TODO: Figure out how to get angular momentum from torque and do that like we did with force
    // Likely need to multiply it by the moment of inertia tensor somehow
    angular_momentum += torque;

    // Get both velocities
    glm::vec3 linear_velocity;
    linear_velocity.x = linear_momentum.x / mass;
    linear_velocity.y = linear_momentum.y / mass;
    linear_velocity.z = linear_momentum.z / mass;
    linear_velocity *= delta;

    orientation = glm::normalize(orientation);
    glm::mat3 R = glm::toMat3(orientation);
    glm::mat3 inertia_inv = R * IbodyInv * glm::transpose(R);

    glm::vec3 angular_velocity = inertia_inv * angular_momentum;
    angular_velocity *= delta;

    glm::quat angular_vel_quat = glm::quat(0.0f, angular_velocity);
    glm::quat orientation_delta = angular_vel_quat * orientation;
    orientation_delta *= 0.5;

    position += linear_velocity;
    orientation = glm::normalize(orientation_delta + (0.5f * orientation));

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