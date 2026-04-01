#include "physics.h"
#include "dynamics.h"
#include <iostream>

PhysicsWorld::PhysicsWorld()
{

}

BodyId PhysicsWorld::create_body(const PhysicsShape& shape, Real mass, PhysicsLayer layer)
{
    BodyId id = bodies.size();
    bodies.push_back(PhysicsBody(shape, mass, layer));
    return id;
}

BodyId PhysicsWorld::create_body(const PhysicsShape& shape, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer)
{
    BodyId id = bodies.size();
    bodies.push_back(PhysicsBody(shape, position, orientation, mass, layer));
    return id;
}

BodyId PhysicsWorld::create_body(const PhysicsShape& shape, const PhysicsMaterial& material, Real mass, PhysicsLayer layer)
{
    BodyId id = bodies.size();
    bodies.push_back(PhysicsBody(shape, material, mass, layer));
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

    Affine3 mat = Affine3::Identity();
    mat.translate(body.transform.position);
    mat.rotate(body.transform.orientation);

    return mat.matrix();
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

CollisionResult PhysicsWorld::check_collision(PhysicsBody* a, PhysicsBody* b)
{
    // Sort by shape type
    if (a->shape.type > b->shape.type)
    {
        PhysicsBody* temp = a;
        a = b;
        b = temp;
    }

    // Call correct function depending on a and b's types
    return collision_funcs[a->shape.type][b->shape.type](&a->shape, &a->transform, &b->shape, &b->transform); 
}

CollisionResult PhysicsWorld::check_sphere_sphere_collision(const PhysicsShape* const a, const Transform* const at, const PhysicsShape* const b, const Transform* const bt)
{
    Vector3 position_diff = bt->position - at->position;

    if ( std::abs(position_diff.norm()) < (a->sphere.radius + b->sphere.radius))
    {
        return CollisionResult {
            .colliding = true,
            .norm = position_diff
        };
    }
    return CollisionResult { .colliding = false };
}

CollisionResult PhysicsWorld::check_sphere_plane_collision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const plane, const Transform* const plane_transform)
{
    Vector3 plane_norm = plane_transform->orientation * Vector3(0.0, 1.0, 0.0);
    plane_norm.normalize();

    // Check collision using mathematical formula
    Real norm_projection = plane_norm.dot((sphere_transform->position - plane_transform->position)) / plane_norm.norm();
    
    if (std::abs(norm_projection) < sphere->sphere.radius)
    {
        return CollisionResult {
            .colliding = true,
            .norm = plane_norm,
        };
    } 

    return CollisionResult{.colliding = false};
}

CollisionResult PhysicsWorld::check_plane_plane_collision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform)
{
    //NOT_IMPLEMENTED();
    return {};
}

CollisionResult PhysicsWorld::check_sphere_obb_collision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const obb, const Transform* const obb_transform)
{
    // NOT_IMPLEMENTED();
    return {};
}

CollisionResult PhysicsWorld::check_plane_obb_collision(const PhysicsShape* const plane, const Transform* const plane_transform, const PhysicsShape* const obb, const Transform* const obb_transform)
{
    // NOT_IMPLEMENTED();
    return {};
}


CollisionResult PhysicsWorld::check_obb_obb_collision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform)
{
    const OBBShape* a_shape = &a->obb;
    const OBBShape* b_shape = &b->obb;
    
    const float EPSILON = 1e-6f;

    Matrix3 a_axis = a_transform->orientation.toRotationMatrix();
    Matrix3 b_axis = b_transform->orientation.toRotationMatrix();

    Matrix3 rotation;
    Matrix3 abs_rotation;

    // Compute rotation which represents the rotation of b in terms of a's coordinate space
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rotation(i, j) = a_axis.col(i).dot(b_axis.col(j));
            abs_rotation(i, j) = std::abs(rotation(i, j)) + EPSILON;
        }
    }

    // Get the translation of b from a's center and put it in terms of a's coordinate system
    Vector3 translation = b_transform->position - a_transform->position;
    translation = Vector3(a_axis.col(0).dot(translation), a_axis.col(1).dot(translation), a_axis.col(2).dot(translation));

    float ra, rb;

    // Test axes of a
    for (int i = 0; i < 3; i++)
    {
        ra = a_shape->half_extent(i);
        rb = b_shape->half_extent.x() * abs_rotation(i, 0) + b_shape->half_extent.y() * abs_rotation(i, 1) + b_shape->half_extent.z() * abs_rotation(i, 2);
        if (std::abs(translation(i)) > ra + rb) return CollisionResult {
            .colliding = false
        };
    }


    // Test axes of b
    for (int i = 0; i < 3; i++)
    {
        ra = a_shape->half_extent.x() * abs_rotation(0, i) + a_shape->half_extent.y() * abs_rotation(1, i) + a_shape->half_extent.z() * abs_rotation(2, i);
        rb = b_shape->half_extent(i);
        if ( std::abs(translation.x() * rotation(0, i) + translation.y() * rotation(1, i) + translation.z() * rotation(2, i)) > ra + rb) return CollisionResult {
            .colliding = false
        };
    }


    // Test cross axes

    ra = a_shape->half_extent.y() * abs_rotation(2, 0) + a_shape->half_extent.z() * abs_rotation(1, 0);
    rb = b_shape->half_extent.y() * abs_rotation(0, 2) + b_shape->half_extent.z() * abs_rotation(0, 1);
    if (std::abs(translation.z() * rotation(1, 0) - translation.y() * rotation(2, 0)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.y() * abs_rotation(2, 1) + a_shape->half_extent.z() * abs_rotation(1, 1);
    rb = b_shape->half_extent.x() * abs_rotation(0, 2) + b_shape->half_extent.z() * abs_rotation(0, 0);
    if (std::abs(translation.z() * rotation(1, 1) - translation.y() * rotation(2, 1)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.y() * abs_rotation(2, 2) + a_shape->half_extent.z() * abs_rotation(1, 2);
    rb = b_shape->half_extent.x() * abs_rotation(0, 1) + b_shape->half_extent.y() * abs_rotation(0, 0);
    if (std::abs(translation.z() * rotation(1, 2) - translation.y() * rotation(2, 2)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(2, 0) + a_shape->half_extent.z() * abs_rotation(0, 0);
    rb = b_shape->half_extent.y() * abs_rotation(1, 2) + b_shape->half_extent.z() * abs_rotation(1, 1);
    if (std::abs(translation.x() * rotation(2, 0) - translation.z() * rotation(0, 0)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(2, 1) + a_shape->half_extent.z() * abs_rotation(0, 1);
    rb = b_shape->half_extent.x() * abs_rotation(1, 2) + b_shape->half_extent.z() * abs_rotation(1, 0);
    if (std::abs(translation.x() * rotation(2, 1) - translation.z() * rotation(0, 1)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(2, 2) + a_shape->half_extent.z() * abs_rotation(0, 2);
    rb = b_shape->half_extent.x() * abs_rotation(1, 1) + b_shape->half_extent.y() * abs_rotation(1, 0);
    if (std::abs(translation.x()* rotation(2, 2) - translation.z() * rotation(0, 2)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(1, 0) + a_shape->half_extent.y() * abs_rotation(0, 0);
    rb = b_shape->half_extent.y() * abs_rotation(2, 2) + b_shape->half_extent.z() * abs_rotation(2, 1);
    if (std::abs(translation.y() * rotation(0, 0) - translation.x() * rotation(1, 0)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(1, 1) + a_shape->half_extent.y() * abs_rotation(0, 1);
    rb = b_shape->half_extent.x() * abs_rotation(2, 2) + b_shape->half_extent.z() * abs_rotation(2, 0);
    if (std::abs(translation.y() * rotation(0, 1) - translation.x() * rotation(1, 1)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(1, 2) + a_shape->half_extent.y() * abs_rotation(0, 2);
    rb = b_shape->half_extent.x() * abs_rotation(2, 1) + b_shape->half_extent.y() * abs_rotation(2, 0);
    if (std::abs(translation.y() * rotation(0, 2) - translation.x() * rotation(1, 2)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    return CollisionResult {
        .colliding = true
    };
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
                // Vector3 reflected_vec = bodies[i].linear_momentum - 2.0f * bodies[i].linear_momentum.dot(result.norm) * result.norm;
                // bodies[i].linear_momentum = reflected_vec * (bodies[i].material.restitution + bodies[j].material.restitution) / 2.0f;
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

            // Update position and orientation based on everything (convert angular velocity to quaternion)
            body.transform.position += body.transform.orientation * getLinearFromSpatial(body.velocity) * delta;

            // Get the delta quaternion
            Vector3 omega = body.transform.orientation * getAngularFromSpatial(body.velocity);
            Real omega_magnitude = omega.norm();
            Quaternion delta_q = Quaternion(cos(omega_magnitude * delta / 2.0), omega.normalized() * sin(omega_magnitude * delta / 2.0));

            // Update orientation based on delta_q
            body.transform.orientation = delta_q * body.transform.orientation;
            body.transform.orientation.normalize();
        }
    }


    // for (PhysicsBody& body : bodies)
    // {
    //     // Update body position
    //     if (body.layer == PhysicsLayer::DYNAMIC)
    //     {
 
    //         // Gravity = acceleration (-9.8) * mass
    //         Vector3 gravity = grav_acceleration;
    //         gravity *= body.mass;
    //         body.add_force(gravity);

    //         // Move this outside of the physics body.
    //         // Body shouldn't be responsible for stepping itself
    //         body.step(delta); 
    //     }
    // }
}

void PhysicsWorld::set_gravity(const Vector6& grav)
{
    this->grav_acceleration = grav;
}