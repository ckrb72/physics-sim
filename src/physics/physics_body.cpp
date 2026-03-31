#include "physics.h"

PhysicsBody::PhysicsBody(const PhysicsShape& shape, const PhysicsMaterial& material, Real mass, PhysicsLayer layer)
:shape(shape), mass(mass), transform(Vector3(0.0, 0.0, 0.0), Quaternion(1.0, 0.0, 0.0, 0.0)), layer(layer), material(material)
{
    spatial_inertia = GetSpatialInertia(shape, mass);
}

PhysicsBody::PhysicsBody(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer)
:shape(shape), transform(position, orientation), mass(mass), layer(layer), material(material)
{
    spatial_inertia = GetSpatialInertia(shape, mass);
}

PhysicsBody::PhysicsBody(const PhysicsShape& shape, Real mass, PhysicsLayer layer)
:shape(shape), mass(mass), transform(Vector3(0.0, 0.0, 0.0), Quaternion(1.0, 0.0, 0.0, 0.0)), layer(layer), material({})
{
    spatial_inertia = GetSpatialInertia(shape, mass);
}

PhysicsBody::PhysicsBody(const PhysicsShape& shape, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer)
:shape(shape), transform(position, orientation), mass(mass), layer(layer), material({})
{
    spatial_inertia = GetSpatialInertia(shape, mass);
}