#include "physics.h"
#include "dynamics.h"
#include <iostream>

static Matrix4 get_transform_matrix(const Transform& transform)
{
    Affine3 mat = Affine3::Identity();
    mat.translate(transform.position);
    mat.rotate(transform.orientation);
    return mat.matrix();
}

BodyId PhysicsWorld::create_body(const PhysicsShape& shape, Real mass, PhysicsLayer layer)
{
    BodyId id = bodies.size();
    bodies.push_back(PhysicsBody(shape, PhysicsMaterial{}, Vector3::Zero(), Quaternion::Identity(), mass, layer));
    return id;
}
BodyId PhysicsWorld::create_body(const PhysicsShape& shape, const Vector3& position, Real mass, PhysicsLayer layer)
{
    BodyId id = bodies.size();
    bodies.push_back(PhysicsBody(shape, PhysicsMaterial{}, position, Quaternion::Identity(), mass, layer));
    return id;
}

BodyId PhysicsWorld::create_body(const PhysicsShape& shape, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer)
{
    BodyId id = bodies.size();
    bodies.push_back(PhysicsBody(shape, PhysicsMaterial{}, position, orientation, mass, layer));
    return id;
}

BodyId PhysicsWorld::create_body(const PhysicsShape& shape, const PhysicsMaterial& material, Real mass, PhysicsLayer layer)
{
    BodyId id = bodies.size();
    bodies.push_back(PhysicsBody(shape, material, Vector3::Zero(), Quaternion::Identity(), mass, layer));
    return id;
}

BodyId PhysicsWorld::create_body(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, Real mass, PhysicsLayer layer)
{
    BodyId id = bodies.size();
    bodies.push_back(PhysicsBody(shape, material, position, Quaternion::Identity(), mass, layer));
    return id;
}

BodyId PhysicsWorld::create_body(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer)
{
    BodyId id = bodies.size();
    bodies.push_back(PhysicsBody(shape, material, position, orientation, mass, layer));
    return id;
}

Matrix4 PhysicsWorld::get_world_matrix(BodyId id)
{
    if (id < 0 || id > bodies.size() - 1) return {};

    const PhysicsBody& body = bodies[id];
    return get_transform_matrix(body.transform);
}

void PhysicsWorld::set_linear_velocity(BodyId id, const Vector3& v)
{
    if (id < 0 || id > bodies.size() - 1) return;

    PhysicsBody& body = bodies[id];
    body.velocity.segment<3>(3) = body.transform.orientation.inverse() * v;
}

void PhysicsWorld::set_angular_velocity(BodyId id, const Vector3& omega)
{
    if (id < 0 || id > bodies.size() - 1) return;
    
    PhysicsBody& body = bodies[id];
    body.velocity.segment<3>(0) = body.transform.orientation.inverse() * omega;
}

bool PhysicsWorld::is_colliding(BodyId a, BodyId b)
{
    if (a == b) return false;

    if (a > bodies.size() - 1 || b > bodies.size() - 1) return false;

    CollisionResult result = check_collision(&bodies[a], &bodies[b]);
    return result.colliding;
}
       

// TODO: Make it so update runs multiple steps if delta > 1 / 60
void PhysicsWorld::update(Real delta)
{
    // Check body collisions and update forces appropriately

    for (int i = 0; i < bodies.size(); i++)
    {
        for (int j = 0; j < bodies.size(); j++)
        {
            if (i == j || (bodies[i].layer == PhysicsLayer::STATIC && bodies[j].layer == PhysicsLayer::STATIC)) continue;

            CollisionResult result = check_collision(&bodies[i], &bodies[j]);
            if (result.colliding) 
            {
                result.norm.normalize();
                Vector3 world_linear_velocity = bodies[i].transform.orientation * getLinearFromSpatial(bodies[i].velocity);
                Vector3 reflected_vec = world_linear_velocity - 2.0f * world_linear_velocity.dot(result.norm) * result.norm;
                bodies[i].velocity.segment<3>(3) = reflected_vec * (bodies[i].material.restitution + bodies[j].material.restitution) / 2.0f;
            }

        }
    }

    for (PhysicsBody& body : bodies)
    {
        // Only update bodies that are dynamic (i.e. respond to physics events)
        // Static stays still
        // Kinematic moves along a predefined curve (like an animation) and doesn't have forces knock it off course
        if (body.layer == PhysicsLayer::DYNAMIC)
        {
            Vector6 acceleration = calculateForwardDynamics({.velocity = body.velocity, .spatial_inertia = body.spatial_inertia}, grav_acceleration * body.mass);

            // Update velocity based on acceleration
            body.velocity += acceleration * delta;

            // Update position (converting linear velocity from body space to world space by using transform orientation)
            body.transform.position += body.transform.orientation * getLinearFromSpatial(body.velocity) * delta;

            // Get the delta quaternion (converting angular velocity from body space to world space by using transform orientation)
            Vector3 omega = body.transform.orientation * getAngularFromSpatial(body.velocity);
            Real omega_magnitude = omega.norm();
            Quaternion delta_q = Quaternion(cos(omega_magnitude * delta / 2.0), omega.normalized() * sin(omega_magnitude * delta / 2.0));

            // Update orientation based on delta_q
            body.transform.orientation = delta_q * body.transform.orientation;
            body.transform.orientation.normalize();

            // body.force = Vector3::Zero();
            // body.torque = Vector3::Zero();
        }
    }
}

void PhysicsWorld::set_gravity(const Vector6& grav)
{
    this->grav_acceleration = grav;
}