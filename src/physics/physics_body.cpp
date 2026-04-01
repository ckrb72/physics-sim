#include "physics.h"

PhysicsBody::PhysicsBody(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer)
:shape(shape), material(material), transform(position, orientation), mass(mass), layer(layer)
{
    spatial_inertia = GetSpatialInertia(shape, mass);
}