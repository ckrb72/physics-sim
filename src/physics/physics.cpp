#include "physics.h"
#include <util/util.h>

    /*
    const double mass = 10.0;
    glm::vec3 dimensions = {1.0, 1.0, 1.0};

    const glm::mat3 Ibody = 
    {
        {mass / 12.0 * ( (dimensions[1] * dimensions[1]) + (dimensions[2] * dimensions[2]) ), 0.0, 0.0},
        {0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[2] * dimensions[2]) ), 0.0},
        {0.0, 0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[1] * dimensions[1]) )}
    }; 

    const double radius = 1.0;

    const double inertia_scale = (2.0 / 5.0) * mass * radius * radius;

    const glm::mat3 Ibody = 
    {
        { inertia_scale, 0.0, 0.0 },
        { 0.0, inertia_scale, 0.0 },
        { 0.0, 0.0, inertia_scale }
    };
    */

SphereShape::SphereShape(double radius)
:r(radius)
{
    this->type = ShapeType::SPHERE;
}


glm::mat3 SphereShape::get_body_mat(double mass)
{
    double inertia_scale = (2.0 / 5.0) * mass * r * r;
    return glm::mat3{
        {inertia_scale, 0.0, 0.0},
        {0.0, inertia_scale, 0.0},
        {0.0, 0.0, inertia_scale}
    };
}

AABBox SphereShape::get_aabb()
{
    NOT_IMPLEMENTED();
    return AABBox{};
}

PlaneShape::PlaneShape(const glm::vec3& extent)
:extent(extent)
{
    this->type = ShapeType::PLANE;
}

glm::mat3 PlaneShape::get_body_mat(double mass)
{
    NOT_IMPLEMENTED();
    glm::vec3 dimensions;
    dimensions.x = extent.x * 2.0;
    dimensions.y = extent.y * 2.0;
    dimensions.z = extent.z * 2.0;

    return {
        {1.0 / 12.0 * ( (dimensions[1] * dimensions[1]) + (dimensions[2] * dimensions[2]) ), 0.0, 0.0},
        {0.0, 1.0 / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[2] * dimensions[2]) ), 0.0},
        {0.0, 0.0, 1.0 / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[1] * dimensions[1]) )}
    };
}

AABBox PlaneShape::get_aabb()
{
    NOT_IMPLEMENTED();
    return AABBox{};
}

OBBShape::OBBShape(const glm::vec3& half_extent)
:half_extent(half_extent)
{
    this->type = ShapeType::OBB;
}

glm::mat3 OBBShape::get_body_mat(double mass)
{
    glm::vec3 dimensions;
    dimensions.x = half_extent.x * 2.0;
    dimensions.y = half_extent.y * 2.0;
    dimensions.z = half_extent.z * 2.0;

    return {
        {mass / 12.0 * ( (dimensions[1] * dimensions[1]) + (dimensions[2] * dimensions[2]) ), 0.0, 0.0},
        {0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[2] * dimensions[2]) ), 0.0},
        {0.0, 0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[1] * dimensions[1]) )}
    };
}

AABBox OBBShape::get_aabb()
{
    NOT_IMPLEMENTED();
    return {};
}