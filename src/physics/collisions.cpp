/*
    Holds all collision algorithms used by the collision detection system
*/

// TODO: FIX ALL NORMAL DIRECTIONS

#include "physics.h"
#include "dynamics.h"

bool PhysicsWorld::isColliding(BodyID a, BodyID b)
{
    if (a == b) return false;

    if (a > bodies.size() - 1 || b > bodies.size() - 1) return false;

    CollisionQuery result = checkCollision(&bodies[a], &bodies[b]);
    return result.colliding;
}

CollisionQuery PhysicsWorld::checkCollision(const PhysicsBody* a, const PhysicsBody* b)
{

    bool swapped = false;
    // Sort by shape type
    if (a->shape.type > b->shape.type)
    {
        const PhysicsBody* temp = a;
        a = b;
        b = temp;
        swapped = true;
    }

    // if the shapes were swapped, flip the normal
    // (this assumes that the normal will always point from the first shape to the second shape which is probably what we want)

    // Call correct function depending on a and b's types
    CollisionQuery result = collision_funcs[a->shape.type][b->shape.type](&a->shape, &a->transform, &b->shape, &b->transform); 
    if (swapped)
    {
        result.norm = -result.norm;
    }

    return result;
}

void PhysicsWorld::handleCollision(const Collision& collision)
{
    // Push the object away from the other object
    if (bodies[collision.a].layer == PhysicsLayer::DYNAMIC)
    {
        bodies[collision.a].transform.position -= collision.depth * collision.norm;
        // Reflect the object across the normal
        Vector3 world_linear_velocity = bodies[collision.a].transform.orientation * getLinearFromSpatial(bodies[collision.a].velocity);
        Vector3 reflected_vec = world_linear_velocity - 2.0f * world_linear_velocity.dot(collision.norm) * collision.norm;
        bodies[collision.a].velocity.segment<3>(3) = reflected_vec * (bodies[collision.a].material.restitution + bodies[collision.b].material.restitution) / 2.0f;
    }

    if (bodies[collision.b].layer == PhysicsLayer::DYNAMIC)
    {
        bodies[collision.b].transform.position += collision.depth * collision.norm;
        Vector3 world_linear_velocity_b = bodies[collision.b].transform.orientation * getLinearFromSpatial(bodies[collision.b].velocity);
        Vector3 reflected_b_vec = world_linear_velocity_b - 2.0f * world_linear_velocity_b.dot(collision.norm) * collision.norm;
        bodies[collision.b].velocity.segment<3>(3) = reflected_b_vec * (bodies[collision.a].material.restitution + bodies[collision.b].material.restitution) / 2.0f;
    }
}

CollisionQuery PhysicsWorld::checkSphereSphereCollision(const PhysicsShape* const a, const Transform* const at, const PhysicsShape* const b, const Transform* const bt)
{
    Vector3 position_diff = bt->position - at->position;
    Real distance = position_diff.norm();
    Real radius_sum = a->sphere.radius + b->sphere.radius;

    if (distance <= radius_sum)
    {
        return CollisionQuery {
            .colliding = true,
            .norm = position_diff.normalized(),
            .depth = radius_sum - distance,
            .point = Vector3::Zero()
        };
    }
    return CollisionQuery { .colliding = false };
}

CollisionQuery PhysicsWorld::checkSpherePlaneCollision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const plane, const Transform* const plane_transform)
{
    Vector3 plane_norm = plane_transform->orientation * Vector3(0.0, 1.0, 0.0);
    plane_norm.normalize();

    // Check collision using mathematical formula (don't need to divide by plane_norm.norm() because plane_norm is already normalized)
    Real norm_projection = plane_norm.dot((sphere_transform->position - plane_transform->position));
    Real distance = std::abs(norm_projection);

    if (distance <= sphere->sphere.radius)
    {
        return CollisionQuery {
            .colliding = true,
            .norm = (norm_projection > 0.0) ? -plane_norm : plane_norm,
            .depth = sphere->sphere.radius - distance,
            .point = Vector3::Zero()
        };
    } 

    return CollisionQuery { .colliding = false };
}

CollisionQuery PhysicsWorld::checkPlanePlaneCollision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform)
{
    // NOT_IMPLEMENTED();
    return {};
}

CollisionQuery PhysicsWorld::checkSphereOBBCollision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const obb, const Transform* const obb_transform)
{
    // Transform sphere's position into obb's coordinate space using inverse obb transform
    // Can either multiply the sphere's position by the inverse of the obb transform (convert sphere position to vec4(sphere_pos, 1.0))
    // or can use the transform translations and rotation directly. Subtract the translations, and multiply sphere_pos by inverse of the rotation quaternion
    Vector3 sphere_obbspace_position = obb_transform->orientation.inverse() * (sphere_transform->position - obb_transform->position);

    // Vector3 obb_min = -obb->obb.half_extent;
    // Vector3 obb_max = obb->obb.half_extent;
    Vector3 half_extent = obb->obb.half_extent;

    Vector3 intersection_point;
    intersection_point[0] = std::max(-half_extent[0], std::min(sphere_obbspace_position[0], half_extent[0]));
    intersection_point[1] = std::max(-half_extent[1], std::min(sphere_obbspace_position[1], half_extent[1]));
    intersection_point[2] = std::max(-half_extent[2], std::min(sphere_obbspace_position[2], half_extent[2]));

    Vector3 intersection_diff = sphere_obbspace_position - intersection_point;
    Real distance = intersection_diff.norm();

    if (distance <= sphere->sphere.radius)
    {
        Vector3 norm = -(obb_transform->orientation * intersection_diff);

        return CollisionQuery{
            .colliding = true,
            .norm = norm.normalized(),
            .depth = distance,
            .point = Vector3::Zero()
        };
    }

    return CollisionQuery{ .colliding = false };
}

CollisionQuery PhysicsWorld::checkPlaneOBBCollision(const PhysicsShape* const plane, const Transform* const plane_transform, const PhysicsShape* const obb, const Transform* const obb_transform)
{
    Matrix3 rotation_axes = obb_transform->orientation.toRotationMatrix();
    Vector3 half_extent = obb->obb.half_extent;

    Vector3 plane_norm = plane_transform->orientation * Vector3(0.0, 1.0, 0.0);
    plane_norm.normalize();

    Real projected_radius = half_extent[0] * std::abs(plane_norm.dot(rotation_axes.col(0)))
                          + half_extent[1] * std::abs(plane_norm.dot(rotation_axes.col(1)))
                          + half_extent[2] * std::abs(plane_norm.dot(rotation_axes.col(2)));

    Real signed_distance = std::abs(plane_norm.dot(obb_transform->position - plane_transform->position));
    Real distance = std::abs(signed_distance);

    if (distance <= projected_radius)
    {
        return CollisionQuery{
            .colliding = true,
            .norm = (signed_distance > 0.0) ? plane_norm : -plane_norm,
            .depth = projected_radius - distance,
            .point = Vector3::Zero()
        };
    }

    return CollisionQuery{ .colliding = false };
}


CollisionQuery PhysicsWorld::checkOBBOBBCollision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform)
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
        if (std::abs(translation(i)) > ra + rb) return CollisionQuery {
            .colliding = false
        };
    }


    // Test axes of b
    for (int i = 0; i < 3; i++)
    {
        ra = a_shape->half_extent.x() * abs_rotation(0, i) + a_shape->half_extent.y() * abs_rotation(1, i) + a_shape->half_extent.z() * abs_rotation(2, i);
        rb = b_shape->half_extent(i);
        if ( std::abs(translation.x() * rotation(0, i) + translation.y() * rotation(1, i) + translation.z() * rotation(2, i)) > ra + rb) return CollisionQuery {
            .colliding = false
        };
    }


    // Test cross axes

    ra = a_shape->half_extent.y() * abs_rotation(2, 0) + a_shape->half_extent.z() * abs_rotation(1, 0);
    rb = b_shape->half_extent.y() * abs_rotation(0, 2) + b_shape->half_extent.z() * abs_rotation(0, 1);
    if (std::abs(translation.z() * rotation(1, 0) - translation.y() * rotation(2, 0)) > ra + rb) return CollisionQuery {
        .colliding = false
    };

    ra = a_shape->half_extent.y() * abs_rotation(2, 1) + a_shape->half_extent.z() * abs_rotation(1, 1);
    rb = b_shape->half_extent.x() * abs_rotation(0, 2) + b_shape->half_extent.z() * abs_rotation(0, 0);
    if (std::abs(translation.z() * rotation(1, 1) - translation.y() * rotation(2, 1)) > ra + rb) return CollisionQuery {
        .colliding = false
    };

    ra = a_shape->half_extent.y() * abs_rotation(2, 2) + a_shape->half_extent.z() * abs_rotation(1, 2);
    rb = b_shape->half_extent.x() * abs_rotation(0, 1) + b_shape->half_extent.y() * abs_rotation(0, 0);
    if (std::abs(translation.z() * rotation(1, 2) - translation.y() * rotation(2, 2)) > ra + rb) return CollisionQuery {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(2, 0) + a_shape->half_extent.z() * abs_rotation(0, 0);
    rb = b_shape->half_extent.y() * abs_rotation(1, 2) + b_shape->half_extent.z() * abs_rotation(1, 1);
    if (std::abs(translation.x() * rotation(2, 0) - translation.z() * rotation(0, 0)) > ra + rb) return CollisionQuery {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(2, 1) + a_shape->half_extent.z() * abs_rotation(0, 1);
    rb = b_shape->half_extent.x() * abs_rotation(1, 2) + b_shape->half_extent.z() * abs_rotation(1, 0);
    if (std::abs(translation.x() * rotation(2, 1) - translation.z() * rotation(0, 1)) > ra + rb) return CollisionQuery {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(2, 2) + a_shape->half_extent.z() * abs_rotation(0, 2);
    rb = b_shape->half_extent.x() * abs_rotation(1, 1) + b_shape->half_extent.y() * abs_rotation(1, 0);
    if (std::abs(translation.x()* rotation(2, 2) - translation.z() * rotation(0, 2)) > ra + rb) return CollisionQuery {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(1, 0) + a_shape->half_extent.y() * abs_rotation(0, 0);
    rb = b_shape->half_extent.y() * abs_rotation(2, 2) + b_shape->half_extent.z() * abs_rotation(2, 1);
    if (std::abs(translation.y() * rotation(0, 0) - translation.x() * rotation(1, 0)) > ra + rb) return CollisionQuery {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(1, 1) + a_shape->half_extent.y() * abs_rotation(0, 1);
    rb = b_shape->half_extent.x() * abs_rotation(2, 2) + b_shape->half_extent.z() * abs_rotation(2, 0);
    if (std::abs(translation.y() * rotation(0, 1) - translation.x() * rotation(1, 1)) > ra + rb) return CollisionQuery {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(1, 2) + a_shape->half_extent.y() * abs_rotation(0, 2);
    rb = b_shape->half_extent.x() * abs_rotation(2, 1) + b_shape->half_extent.y() * abs_rotation(2, 0);
    if (std::abs(translation.y() * rotation(0, 2) - translation.x() * rotation(1, 2)) > ra + rb) return CollisionQuery {
        .colliding = false
    };

    // FIXME: FIND OUT HOW TO CALCULATE DEPTH AND NORM
    return CollisionQuery {
        .colliding = true,
        .norm = Vector3::Identity(),
        .depth = 0.0,
        .point = Vector3::Zero()
    };
}