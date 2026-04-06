#include "physics.h"
#include <iostream>

static Eigen::Matrix<Real, 6, 6> GetSphereSpatialInertia(const SphereShape& sphere, Real mass)
{
    Real inertia_scalar = (2.0 / 5.0) * mass * sphere.radius * sphere.radius;
    Eigen::Matrix<Real, 6, 6> spatial_inertia;
    spatial_inertia << inertia_scalar, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, inertia_scalar, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, inertia_scalar, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, mass, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, mass, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, mass;
    return spatial_inertia;
}

static Eigen::Matrix<Real, 6, 6> GetPlaneSpatialInertia(const PlaneShape& plane, Real mass)
{
    Eigen::Matrix<Real, 6, 6> spatial_inertia = Eigen::Matrix<Real, 6, 6>::Identity();

    return spatial_inertia;
}

static Eigen::Matrix<Real, 6, 6> GetOBBSpatialInertia(const OBBShape& obb, Real mass)
{
    Vector3 extents = obb.half_extent * 2.0;
    Real scalar = (1.0 / 12.0) * mass;
    Eigen::Matrix<Real, 6, 6> spatial_inertia;
    spatial_inertia << scalar * (extents[1] * extents[1] + extents[2] * extents[2]), 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, scalar * (extents[0] * extents[0] + extents[2] * extents[2]), 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, scalar * (extents[0] * extents[0] + extents[1] * extents[1]), 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, mass, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, mass, 0.0, 
                       0.0, 0.0, 0.0, 0.0, 0.0, mass;
    return spatial_inertia;
}

Eigen::Matrix<Real, 6, 6> GetSpatialInertia(const PhysicsShape& shape, Real mass)
{
    switch(shape.type)
    {
        case ShapeType::SPHERE:
            return GetSphereSpatialInertia(shape.sphere, mass);
            break;
        case ShapeType::PLANE:
            return GetPlaneSpatialInertia(shape.plane, mass);
            break;
        case ShapeType::OBB:
            return GetOBBSpatialInertia(shape.obb, mass);
            break;
        default:
            return Eigen::Matrix<Real, 6, 6>::Identity();
    }

    return Eigen::Matrix<Real, 6, 6>::Identity();
}

static Matrix3 GetSphereInertiaTensor(const SphereShape& sphere, Real mass)
{
    Real inertia_scalar = (2.0 / 5.0) * mass * sphere.radius * sphere.radius;
    Matrix3 inertia_tensor;
    inertia_tensor << inertia_scalar, 0.0, 0.0,
                      0.0, inertia_scalar, 0.0,
                      0.0, 0.0, inertia_scalar;
    return inertia_tensor;
}

static Matrix3 GetPlaneInertiaTensor(const PlaneShape& plane, Real mass)
{
    std::cout << "Plane Inertia Tensor Not Implemented: " << __FILE__ << ":" << __LINE__ << std::endl;
    return Matrix3::Identity();
}

static Matrix3 GetOBBInertiaTensor(const OBBShape& obb, Real mass)
{
    Vector3 extents = obb.half_extent * 2.0;
    Real scalar = (1.0 / 12.0) * mass;
    Matrix3 inertia_tensor;
    inertia_tensor << scalar * (extents[1] * extents[1] + extents[2] * extents[2]), 0.0, 0.0,
                      0.0, scalar * (extents[0] * extents[0] + extents[2] * extents[2]), 0.0,
                      0.0, 0.0, scalar * (extents[0] * extents[0] + extents[1] * extents[1]);
    return inertia_tensor;
}

Matrix3 GetInertiaTensor(const PhysicsShape& shape, Real mass)
{
    switch(shape.type)
    {
        case ShapeType::SPHERE:
            return GetSphereInertiaTensor(shape.sphere, mass);
            break;
        case ShapeType::PLANE:
            return GetPlaneInertiaTensor(shape.plane, mass);
            break;
        case ShapeType::OBB:
            return GetOBBInertiaTensor(shape.obb, mass);
            break;
        default:
            return Matrix3::Identity();
    }
    return Matrix3::Identity();
}

PhysicsShape PhysicsShape::MakeSphere(Real radius)
{
    return PhysicsShape{
        .type = ShapeType::SPHERE,
        .sphere = SphereShape{
            .radius = radius
        }
    };
}

PhysicsShape PhysicsShape::MakePlane(const Vector3& extent)
{
    return PhysicsShape{
        .type = ShapeType::PLANE,
        .plane = PlaneShape{
            .extent = extent
        }
    };
}

PhysicsShape PhysicsShape::MakeOBB(const Vector3& half_extent)
{
    return PhysicsShape{
        .type = ShapeType::OBB,
        .obb = OBBShape{
            .half_extent = half_extent
        }
    };
}