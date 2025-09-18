#include "physics.h"
#include "util.h"

BoxShape::BoxShape(glm::vec3 half_extent)
:half_extent(half_extent)
{}

bool BoxShape::collide()
{
    NOT_IMPLEMENTED();
    return false;
}

glm::mat3 BoxShape::get_body_mat()
{
    glm::vec3 dimensions;
    dimensions.x = half_extent.x * 2.0;
    dimensions.y = half_extent.y * 2.0;
    dimensions.z = half_extent.z * 2.0;

    return {
        {1.0 / 12.0 * ( (dimensions[1] * dimensions[1]) + (dimensions[2] * dimensions[2]) ), 0.0, 0.0},
        {0.0, 1.0 / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[2] * dimensions[2]) ), 0.0},
        {0.0, 0.0, 1.0 / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[1] * dimensions[1]) )}
    };
}

AABBox BoxShape::get_aabb()
{
    NOT_IMPLEMENTED();
    return AABBox{};
}

SphereShape::SphereShape(double radius)
:r(radius)
{}

bool SphereShape::collide()
{
    NOT_IMPLEMENTED();
    return false;
}

glm::mat3 SphereShape::get_body_mat()
{
    NOT_IMPLEMENTED();
    return glm::mat4(1.0f);
}

AABBox SphereShape::get_aabb()
{
    NOT_IMPLEMENTED();
    return AABBox{};
}

PlaneShape::PlaneShape(const glm::vec3& norm, const glm::vec3& extent)
:norm(norm), extent(extent)
{}

bool PlaneShape::collide()
{
    NOT_IMPLEMENTED();
    return false;
}

glm::mat3 PlaneShape::get_body_mat()
{
    NOT_IMPLEMENTED();
    return glm::mat4(1.0f);
}

AABBox PlaneShape::get_aabb()
{
    NOT_IMPLEMENTED();
    return AABBox{};
}