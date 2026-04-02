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
using BodyID = int32_t;


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

        PhysicsLayer layer = PhysicsLayer::STATIC;
        PhysicsMaterial material;

        friend class PhysicsWorld;

        PhysicsBody(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer);

    public:
        PhysicsBody() = delete;
};


struct CollisionQuery
{
    bool colliding = false;
    Vector3 norm = Vector3::Identity();
    Real depth = 0.0;
    Vector3 point = Vector3::Zero();
};

struct Collision
{
    BodyID a = -1;
    BodyID b = -1;
    Vector3 norm = Vector3::Identity();
    Real depth = 0.0;
    Vector3 point = Vector3::Zero();
};

class PhysicsWorld
{
    private:
        std::vector<PhysicsBody> bodies;
        Vector6 grav_acceleration = Vector6::Zero();

        std::queue<Collision> collisions;

        // For all plane collision algorithms, they just assume the plane is infinite for now
        // Time permitting: take into account plane extents

        static CollisionQuery check_sphere_sphere_collision(const PhysicsShape* const a, const Transform* const at, const PhysicsShape* const b, const Transform* const bt);
        static CollisionQuery check_sphere_plane_collision(const PhysicsShape* const sphere, const Transform* sphere_transform, const PhysicsShape* const plane, const Transform* const plane_transform);
        static CollisionQuery check_sphere_box_collision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const box, const Transform* const box_transform);
        
        static CollisionQuery check_plane_plane_collision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform);
        static CollisionQuery check_plane_box_collision(const PhysicsShape* const plane, const Transform* const plane_transform, const PhysicsShape* const box, const Transform* const box_transform);
        
        static CollisionQuery check_box_box_collision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform);
        
        static CollisionQuery check_sphere_obb_collision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const obb, const Transform* const obb_transform);
        static CollisionQuery check_plane_obb_collision(const PhysicsShape* const plane, const Transform* const plane_transform, const PhysicsShape* const obb, const Transform* const obb_transform);
        static CollisionQuery check_box_obb_collision(const PhysicsShape* const box, const Transform* const box_transform, const PhysicsShape* const obb, const Transform* const obb_transform);
        static CollisionQuery check_obb_obb_collision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform);

        CollisionQuery check_collision(const PhysicsBody* a, const PhysicsBody* b);

        // Array of func pointers for collision tests
        typedef CollisionQuery (*CollisionFunc)(const PhysicsShape* const, const Transform* const, const PhysicsShape* const, const Transform* const);
        CollisionFunc collision_funcs[ShapeType::NUM_SHAPES][ShapeType::NUM_SHAPES] = 
        {
            {nullptr, nullptr, nullptr, nullptr},
            {nullptr, check_sphere_sphere_collision, check_sphere_plane_collision, check_sphere_obb_collision},
            {nullptr, nullptr /*plane sphere*/, check_plane_plane_collision, check_plane_obb_collision},
            {nullptr, nullptr, nullptr, check_obb_obb_collision}
        };

    public:
        PhysicsWorld() {}
        BodyID create_body(const PhysicsShape& shape, Real mass, PhysicsLayer layer);
        BodyID create_body(const PhysicsShape& shape, const Vector3& position, Real mass, PhysicsLayer layer);
        BodyID create_body(const PhysicsShape& shape, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer);
        BodyID create_body(const PhysicsShape& shape, const PhysicsMaterial& material, Real mass, PhysicsLayer layer);
        BodyID create_body(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, Real mass, PhysicsLayer layer);
        BodyID create_body(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer);
        // void remove(BodyID id);

        // Body manipulation functions
        void set_linear_velocity(BodyID id, const Vector3& v);
        void set_angular_velocity(BodyID id, const Vector3& omega);
        
        Matrix4 get_world_matrix(BodyID id);

        // This should be outside of this class but for now it's ok
        bool is_colliding(BodyID a, BodyID b);

        void set_gravity(const Vector6& grav);

        // void set_time_step(Real duration);

        // TODO: Updates with 1 / 60 second granularity. If delta > 1 / 60 the integration step is done multiple times
        void update(Real delta);  // This is where the integration actually occurs
};