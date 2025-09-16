#include "physics.h"

PhysicsBody::PhysicsBody(std::shared_ptr<CollisionShape> collider, double mass)
:collider(collider), mass(mass)
{}

/*
BoxShape::BoxShape(glm::vec3 half_extent)
:half_extent(half_extent)
{}

SphereShape::SphereShape(double radius)
:r(radius)
{}

PlaneShape::PlaneShape(glm::vec3 norm)
:norm(norm)
{}*/

