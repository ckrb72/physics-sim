#include "physics.h"

    /*
    const double mass = 10.0;
    Eigen::Vector3d dimensions = {1.0, 1.0, 1.0};

    const Eigen::Matrix3d Ibody = 
    {
        {mass / 12.0 * ( (dimensions[1] * dimensions[1]) + (dimensions[2] * dimensions[2]) ), 0.0, 0.0},
        {0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[2] * dimensions[2]) ), 0.0},
        {0.0, 0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[1] * dimensions[1]) )}
    }; 

    const double radius = 1.0;

    const double inertia_scale = (2.0 / 5.0) * mass * radius * radius;

    const Eigen::Matrix3d Ibody = 
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


Eigen::Matrix3d SphereShape::get_body_mat(double mass)
{
    double inertia_scale = (2.0 / 5.0) * mass * r * r;
    return Eigen::Matrix3d{
        {inertia_scale, 0.0, 0.0},
        {0.0, inertia_scale, 0.0},
        {0.0, 0.0, inertia_scale}
    };
}

AABBox SphereShape::get_aabb()
{
    // NOT_IMPLEMENTED();
    return AABBox{};
}

PlaneShape::PlaneShape(const Eigen::Vector3d& extent)
:extent(extent)
{
    this->type = ShapeType::PLANE;
}

Eigen::Matrix3d PlaneShape::get_body_mat(double mass)
{
    // NOT_IMPLEMENTED();
    Eigen::Vector3d dimensions;
    dimensions.x() = extent.x() * 2.0;
    dimensions.y() = extent.y() * 2.0;
    dimensions.z() = extent.z() * 2.0;

    Eigen::Matrix3d mat;
    mat << 1.0 / 12.0 * ( (dimensions(1) * dimensions(1)) + (dimensions(2) * dimensions(2)) ), 0.0, 0.0,
    0.0, 1.0 / 12.0 * ( (dimensions(0) * dimensions(0)) + (dimensions(2) * dimensions(2)) ), 0.0,
    0.0, 0.0, 1.0 / 12.0 * ( (dimensions(0) * dimensions(0)) + (dimensions(1) * dimensions(1)) );

    return mat;
}

AABBox PlaneShape::get_aabb()
{
    // NOT_IMPLEMENTED();
    return AABBox{};
}

OBBShape::OBBShape(const Eigen::Vector3d& half_extent)
:half_extent(half_extent)
{
    this->type = ShapeType::OBB;
}

Eigen::Matrix3d OBBShape::get_body_mat(double mass)
{
    Eigen::Vector3d dimensions;
    dimensions.x() = half_extent.x() * 2.0;
    dimensions.y() = half_extent.y() * 2.0;
    dimensions.z() = half_extent.z() * 2.0;

    Eigen::Matrix3d mat;
    mat << mass / 12.0 * ( (dimensions[1] * dimensions[1]) + (dimensions[2] * dimensions[2]) ), 0.0, 0.0,
    0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[2] * dimensions[2]) ), 0.0,
    0.0, 0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[1] * dimensions[1]) );

    return mat;
}

AABBox OBBShape::get_aabb()
{
    // NOT_IMPLEMENTED();
    return {};
}