#include "physics.h"

static Eigen::Matrix<Real, 6, 6> GetSphereSpatialInertia(const PhysicsShape& shape, Real mass)
{
    Real inertia_scalar = (2.0 / 5.0) * mass * shape.sphere.radius * shape.sphere.radius;
    Eigen::Matrix<Real, 6, 6> spatial_inertia;
    spatial_inertia << inertia_scalar, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, inertia_scalar, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, inertia_scalar, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, mass, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, mass, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, mass;
    return spatial_inertia;
}

static Eigen::Matrix<Real, 6, 6> GetPlaneSpatialInertia(const PhysicsShape& shape, Real mass)
{
    Eigen::Matrix<Real, 6, 6> spatial_inertia;

    return spatial_inertia;
}

static Eigen::Matrix<Real, 6, 6> GetOBBSpatialInertia(const PhysicsShape& shape, Real mass)
{
    Vector3 extents = shape.obb.half_extent * 2.0;
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
            return GetSphereSpatialInertia(shape, mass);
            break;
        case ShapeType::PLANE:
            return GetPlaneSpatialInertia(shape, mass);
            break;
        case ShapeType::OBB:
            return GetOBBSpatialInertia(shape, mass);
            break;
        default:
            return {};
    }

    return {};
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