#include "physics.h"

PhysicsBody::PhysicsBody(std::shared_ptr<PhysicsShape> shape, const PhysicsMaterial& material, double mass, PhysicsLayer layer)
:shape(shape), mass(mass), transform(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)), layer(layer), material(material)
{
    Ibody = shape->get_body_mat(mass);
    IbodyInv = Ibody.inverse();
}

PhysicsBody::PhysicsBody(std::shared_ptr<PhysicsShape> shape, const PhysicsMaterial& material, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double mass, PhysicsLayer layer)
:shape(shape), transform(position, orientation), mass(mass), layer(layer), material(material)
{
    Ibody = shape->get_body_mat(mass);
    IbodyInv = Ibody.inverse();
}

PhysicsBody::PhysicsBody(std::shared_ptr<PhysicsShape> shape, double mass, PhysicsLayer layer)
:shape(shape), mass(mass), transform(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)), layer(layer), material({})
{
    Ibody = shape->get_body_mat(mass);
    IbodyInv = Ibody.inverse();
}

PhysicsBody::PhysicsBody(std::shared_ptr<PhysicsShape> shape, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double mass, PhysicsLayer layer)
:shape(shape), transform(position, orientation), mass(mass), layer(layer), material({})
{
    Ibody = shape->get_body_mat(mass);
    IbodyInv = Ibody.inverse();
}



const Eigen::Vector3d& PhysicsBody::get_position() const
{
    return transform.position;
}

const Eigen::Quaterniond& PhysicsBody::get_orientation() const
{
    return transform.orientation;
}

Eigen::Matrix4d PhysicsBody::get_world_matrix() const
{
    Eigen::Affine3d mat = Eigen::Affine3d::Identity();
    mat.translate(transform.position);
    mat.rotate(transform.orientation);
    return mat.matrix();
}

void PhysicsBody::add_force(const Eigen::Vector3d& force)
{
    this->force += force;
}

void PhysicsBody::add_torque(const Eigen::Vector3d& torque)
{
    this->torque += torque;
}

void PhysicsBody::add_impulse(const Eigen::Vector3d& impulse)
{
    this->impulse += impulse;
}

void PhysicsBody::add_torque_impulse(const Eigen::Vector3d& torque_impulse)
{
    this->torque_impulse += torque_impulse;
}

void PhysicsBody::set_linear_velocity(const Eigen::Vector3d& v)
{
    linear_momentum.x() = v.x() * mass;
    linear_momentum.y() = v.y() * mass;
    linear_momentum.z() = v.z() * mass;
}

void PhysicsBody::set_angular_velocity(const Eigen::Vector3d& omega)
{
    transform.orientation.normalize();
    Eigen::Matrix3d rotation = transform.orientation.toRotationMatrix();
    Eigen::Matrix3d inertia_tensor = rotation * Ibody * rotation.transpose().eval();
    angular_momentum = inertia_tensor * omega;
}

void PhysicsBody::set_position(const Eigen::Vector3d& pos)
{
    transform.position = pos;
}

void PhysicsBody::set_orientation(const Eigen::Quaterniond& orientation)
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
    Eigen::Vector3d linear_velocity;
    linear_velocity.x() = linear_momentum.x() / mass;
    linear_velocity.y() = linear_momentum.y() / mass;
    linear_velocity.z() = linear_momentum.z() / mass;

    transform.orientation.normalize();
    Eigen::Matrix3d R = transform.orientation.toRotationMatrix();
    Eigen::Matrix3d inertia_inv = R * IbodyInv * R.transpose().eval();

    Eigen::Vector3d angular_velocity = inertia_inv * angular_momentum;
    Eigen::Quaterniond angular_vel_quat = Eigen::Quaterniond(0.0f, angular_velocity);
    Eigen::Quaterniond orientation_delta = angular_vel_quat * transform.orientation;
    orientation_delta.coeffs() *= 0.5 * delta;

    // Change in position = velocity * time
    Eigen::Vector3d position_delta = linear_velocity;
    position_delta *= delta;

    transform.position += position_delta;
    transform.orientation.coeffs() += (0.5f * transform.orientation.coeffs());
    transform.orientation.normalize();

    force = Eigen::Vector3d(0.0, 0.0, 0.0);
    torque = Eigen::Vector3d(0.0, 0.0, 0.0);
    impulse = Eigen::Vector3d(0.0, 0.0, 0.0);
    torque_impulse = Eigen::Vector3d(0.0, 0.0, 0.0);
}