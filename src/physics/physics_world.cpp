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

BodyID PhysicsWorld::createBody(const PhysicsShape& shape, Real mass, PhysicsLayer layer)
{
    BodyID id = bodies.size();
    bodies.push_back(PhysicsBody(shape, PhysicsMaterial{}, Vector3::Zero(), Quaternion::Identity(), mass, layer));
    return id;
}
BodyID PhysicsWorld::createBody(const PhysicsShape& shape, const Vector3& position, Real mass, PhysicsLayer layer)
{
    BodyID id = bodies.size();
    bodies.push_back(PhysicsBody(shape, PhysicsMaterial{}, position, Quaternion::Identity(), mass, layer));
    return id;
}

BodyID PhysicsWorld::createBody(const PhysicsShape& shape, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer)
{
    BodyID id = bodies.size();
    bodies.push_back(PhysicsBody(shape, PhysicsMaterial{}, position, orientation, mass, layer));
    return id;
}

BodyID PhysicsWorld::createBody(const PhysicsShape& shape, const PhysicsMaterial& material, Real mass, PhysicsLayer layer)
{
    BodyID id = bodies.size();
    bodies.push_back(PhysicsBody(shape, material, Vector3::Zero(), Quaternion::Identity(), mass, layer));
    return id;
}

BodyID PhysicsWorld::createBody(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, Real mass, PhysicsLayer layer)
{
    BodyID id = bodies.size();
    bodies.push_back(PhysicsBody(shape, material, position, Quaternion::Identity(), mass, layer));
    return id;
}

BodyID PhysicsWorld::createBody(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer)
{
    BodyID id = bodies.size();
    bodies.push_back(PhysicsBody(shape, material, position, orientation, mass, layer));
    return id;
}

Matrix4 PhysicsWorld::getWorldMatrix(BodyID id)
{
    if (id < 0 || id > bodies.size() - 1) return {};

    const PhysicsBody& body = bodies[id];
    return get_transform_matrix(body.transform);
}

void PhysicsWorld::setLinearVelocity(BodyID id, const Vector3& v)
{
    if (id < 0 || id > bodies.size() - 1) return;

    PhysicsBody& body = bodies[id];
    body.velocity.segment<3>(3) = body.transform.orientation.inverse() * v;
}

void PhysicsWorld::setAngularVelocity(BodyID id, const Vector3& omega)
{
    if (id < 0 || id > bodies.size() - 1) return;
    
    PhysicsBody& body = bodies[id];
    body.velocity.segment<3>(0) = body.transform.orientation.inverse() * omega;
}
       

// TODO: Make it so update runs multiple steps if delta > 1 / 60
void PhysicsWorld::update(Real delta)
{
    // Check body collisions and update forces appropriately

    // Collision Queries
    queryCollisions();
    for (int i = 0; i < bodies.size(); i++)
    {
        for (int j = i + 1; j < bodies.size(); j++)
        {
            if (bodies[i].layer == PhysicsLayer::STATIC && bodies[j].layer == PhysicsLayer::STATIC) continue;

            CollisionQuery result = checkCollision(&bodies[i], &bodies[j]);
            if (result.colliding) 
            {
                collisions.push(Collision{ .a = i, .b = j, .norm = result.norm, .depth = result.depth, .point = result.point });
            }
        }
    }

    // Collision Resolution
    while(!collisions.empty())
    {
        Collision collision = collisions.front();
        collisions.pop();
        handleCollision(collision);
    }

    // Update forces and positions
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

            body.force = Vector3::Zero();
            body.torque = Vector3::Zero();
        }
    }
}

void PhysicsWorld::setGravity(const Vector6& grav)
{
    this->grav_acceleration = grav;
}