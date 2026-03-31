#pragma once
#include <vector>
#include <queue>
#include <memory>
#include <Eigen/Dense>
#include <cmath>

#define PRECISION_HIGH

#ifdef PRECISION_HIGH
    using Real = double;
    using Vector3 = Eigen::Vector3d;
    using Quaternion = Eigen::Quaterniond;
    using Matrix4 = Eigen::Matrix4d;
    using Matrix3 = Eigen::Matrix3d;
    using Affine3 = Eigen::Affine3d;
#else
    using Real = float;
    using Vector3 = Eigen::Vector3f;
    using Quaternion = Eigen::Quaternionf;
    using Matrix4 = Eigen::Matrix4f;
    using Matrix3 = Eigen::Matrix3f;
    using Affine3 = Eigen::Affine3f;
#endif

using Vector6 = Eigen::Matrix<Real, 6, 1>;
using BodyId = int32_t;


inline Real DegreesToRadians(Real degrees)
{
    return degrees * (M_PI / 180.0);
}

inline std::array<float, 16> EigenMatrixToFloatArray(const Matrix4& mat)
{
    std::array<float, 16> array = {};
    const Real* mat_data = mat.data();
    for (int i = 0; i < 16; i++)
    {
        array[i] = static_cast<float>(mat_data[i]);
    }

    return array;
}


struct AABBox
{
    Vector3 half_extents;
    Vector3 position;
};

enum ShapeType
{
    SHAPE,
    SPHERE,
    PLANE,
    OBB,
    NUM_SHAPES
};

struct PhysicsMaterial
{
    Real restitution = 1.0f;
};

struct SphereShape
{
    Real radius;
};

struct PlaneShape
{
    Vector3 extent;
};

struct OBBShape
{
    Vector3 half_extent;
};

struct PhysicsShape
{
    ShapeType type;

    // In a real engine would want to make each of these into an array and just have an index
    union {
        SphereShape sphere;
        PlaneShape plane;
        OBBShape obb;
    };

    static PhysicsShape MakeSphere(Real radius);
    static PhysicsShape MakePlane(const Vector3& extent);
    static PhysicsShape MakeOBB(const Vector3& half_extent);
};

Eigen::Matrix<Real, 6, 6> GetSpatialInertia(const PhysicsShape& shape, Real mass);

enum PhysicsLayer
{
    DYNAMIC,
    KINEMATIC,
    STATIC
};

struct Transform
{
    Vector3 position = Vector3(0.0, 0.0, 0.0);
    Quaternion orientation = Quaternion(1.0, 0.0, 0.0, 0.0);
};


class PhysicsBody
{
    private:
        Real mass = 0.0;
        PhysicsShape shape;
        Eigen::Matrix<Real, 6, 6> spatial_inertia = {};
        Vector6 velocity = {};
        Transform transform;

        // Add to this each frame to apply forces to the object (converted to a spatial force vector for the forward dynamics pass)
        Vector3 force = Vector3(0.0, 0.0, 0.0);
        Vector3 torque = Vector3(0.0, 0.0, 0.0);

        // Vector3 linear_momentum = Vector3(0.0, 0.0, 0.0);
        // Vector3 angular_momentum = Vector3(0.0, 0.0, 0.0);

        // // Units: N (kgm / s)
        // // Multiplied by delta then added to momentum
        // Vector3 force = Vector3(0.0, 0.0, 0.0);

        // // Units: Ns (kgm)
        // // Added to momentum directly
        // Vector3 impulse = Vector3(0.0, 0.0, 0.0);

        // // Units: Nm (kgm^2 / s)
        // // Multiplied by delta then added to angular momentum
        // Vector3 torque = Vector3(0.0, 0.0, 0.0);

        // // Units: Nms (kgm^2)
        // // Added to angular momentum directly
        // Vector3 torque_impulse = Vector3(0.0, 0.0, 0.0);

        PhysicsLayer layer = PhysicsLayer::STATIC;
        PhysicsMaterial material;

        friend class PhysicsWorld;

        PhysicsBody(const PhysicsShape& shape, const PhysicsMaterial& material, Real mass, PhysicsLayer layer);
        PhysicsBody(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer);
        PhysicsBody(const PhysicsShape& shape, Real mass, PhysicsLayer layer);
        PhysicsBody(const PhysicsShape& shape, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer);

    public:
        PhysicsBody() = delete;
};

struct BodyInfo
{
    Real mass = 0.0;
    Vector3 position = Vector3(0.0, 0.0, 0.0);
    Quaternion orientation = Quaternion(1.0, 0.0, 0.0, 0.0);

    Vector3 linear_momentum = Vector3(0.0, 0.0, 0.0);
    Vector3 angular_momentum = Vector3(0.0, 0.0, 0.0);

    Vector3 force = Vector3(0.0, 0.0, 0.0);
    Vector3 impulse = Vector3(0.0, 0.0, 0.0);
    Vector3 torque = Vector3(0.0, 0.0, 0.0);
    Vector3 torque_impulse = Vector3(0.0, 0.0, 0.0);
};


struct CollisionResult
{
    bool colliding = false;
    Vector3 norm = Vector3(0.0, 0.0, 0.0);
    Real depth = 0.0;
};

class PhysicsWorld
{
    private:
        std::vector<PhysicsBody> bodies;
        Vector6 grav_acceleration = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

        static CollisionResult check_sphere_sphere_collision(const PhysicsShape* const a, const Transform* const at, const PhysicsShape* const b, const Transform* const bt);
        static CollisionResult check_sphere_plane_collision(const PhysicsShape* const sphere, const Transform* sphere_transform, const PhysicsShape* const plane, const Transform* const plane_transform);
        static CollisionResult check_sphere_box_collision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const box, const Transform* const box_transform);
        
        static CollisionResult check_plane_plane_collision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform);
        static CollisionResult check_plane_box_collision(const PhysicsShape* const plane, const Transform* const plane_transform, const PhysicsShape* const box, const Transform* const box_transform);
        
        static CollisionResult check_box_box_collision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform);
        
        static CollisionResult check_sphere_obb_collision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const obb, const Transform* const obb_transform);
        static CollisionResult check_plane_obb_collision(const PhysicsShape* const plane, const Transform* const plane_transform, const PhysicsShape* const obb, const Transform* const obb_transform);
        static CollisionResult check_box_obb_collision(const PhysicsShape* const box, const Transform* const box_transform, const PhysicsShape* const obb, const Transform* const obb_transform);
        static CollisionResult check_obb_obb_collision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform);

        CollisionResult check_collision(PhysicsBody* a, PhysicsBody* b);

        // Array of func pointers for collision tests
        typedef CollisionResult (*CollisionFunc)(const PhysicsShape* const, const Transform* const, const PhysicsShape* const, const Transform* const);
        CollisionFunc collision_funcs[ShapeType::NUM_SHAPES][ShapeType::NUM_SHAPES] = 
        {
            {nullptr, nullptr, nullptr, nullptr},
            {nullptr, check_sphere_sphere_collision, check_sphere_plane_collision, check_sphere_obb_collision},
            {nullptr, nullptr /*plane sphere*/, check_plane_plane_collision, check_plane_obb_collision},
            {nullptr, nullptr, nullptr, check_obb_obb_collision}
        };

    public:
        PhysicsWorld();
        BodyId create_body(const PhysicsShape& shape, Real mass, PhysicsLayer layer);
        BodyId create_body(const PhysicsShape& shape, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer);
        BodyId create_body(const PhysicsShape& shape, const PhysicsMaterial& material, Real mass, PhysicsLayer layer);
        BodyId create_body(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer);
        void remove(BodyId id);

        // Body manipulation functions
        void set_linear_velocity(BodyId id, const Vector3& v);
        void set_angular_velocity(BodyId id, const Vector3& omega);
        Matrix4 get_world_matrix(BodyId id);

        bool is_colliding(BodyId a, BodyId b);

        void set_gravity(const Vector6& grav);

        BodyInfo get_info(BodyId id) const;

        void set_time_step(Real duration);

        // TODO: Updates with 1 / 60 second granularity. If delta > 1 / 60 the integration step is done multiple times
        void update(Real delta);  // This is where the integration actually occurs
};