#pragma once
#include <vector>
#include <deque>
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
Matrix3 GetInertiaTensor(const PhysicsShape& shape, Real mass);

enum PhysicsLayer
{
    DYNAMIC,
    KINEMATIC,
    STATIC
};

struct Transform
{
    Vector3 position = Vector3::Zero();
    Quaternion orientation = Quaternion::Identity();
};


class PhysicsBody
{
    private:
        Real mass = 0.0;
        PhysicsShape shape;
        Eigen::Matrix<Real, 6, 6> spatial_inertia = Eigen::Matrix<Real, 6, 6>::Identity();
        Matrix3 inverse_inertia = Matrix3::Identity();
        Vector6 velocity = Vector6::Zero();
        Transform transform;

        // Add to this each frame to apply forces to the object (converted to a spatial force vector for the forward dynamics pass)
        Vector3 force = Vector3::Zero();
        Vector3 torque = Vector3::Zero();

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

        std::deque<Collision> collisions;

        // For all plane collision algorithms, they just assume the plane is infinite for now
        // Time permitting: take into account plane extents

        static CollisionQuery checkSphereSphereCollision(const PhysicsShape* const a, const Transform* const at, const PhysicsShape* const b, const Transform* const bt);
        static CollisionQuery checkSpherePlaneCollision(const PhysicsShape* const sphere, const Transform* sphere_transform, const PhysicsShape* const plane, const Transform* const plane_transform);
        static CollisionQuery checkSphereBoxCollision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const box, const Transform* const box_transform);
        
        static CollisionQuery checkPlanePlaneCollision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform);
        static CollisionQuery checkPlaneBoxCollision(const PhysicsShape* const plane, const Transform* const plane_transform, const PhysicsShape* const box, const Transform* const box_transform);
        
        static CollisionQuery checkBoxBoxCollision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform);
        
        static CollisionQuery checkSphereOBBCollision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const obb, const Transform* const obb_transform);
        static CollisionQuery checkPlaneOBBCollision(const PhysicsShape* const plane, const Transform* const plane_transform, const PhysicsShape* const obb, const Transform* const obb_transform);
        static CollisionQuery checkBoxOBBCollision(const PhysicsShape* const box, const Transform* const box_transform, const PhysicsShape* const obb, const Transform* const obb_transform);
        static CollisionQuery checkOBBOBBCollision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform);

        const uint32_t collisionPositionIterations =1;
        const uint32_t collisionVelocityIterations = 10;

        CollisionQuery checkCollision(const PhysicsBody* a, const PhysicsBody* b);
        void handleCollisionVelocities(const Collision& collision);
        void handleCollisionPositions(const Collision& collision);

        // Array of func pointers for collision tests
        typedef CollisionQuery (*CollisionFunc)(const PhysicsShape* const, const Transform* const, const PhysicsShape* const, const Transform* const);
        CollisionFunc collision_funcs[ShapeType::NUM_SHAPES][ShapeType::NUM_SHAPES] = 
        {
            {nullptr, nullptr, nullptr, nullptr},
            {nullptr, checkSphereSphereCollision, checkSpherePlaneCollision, checkSphereOBBCollision},
            {nullptr, nullptr /*plane sphere*/, checkPlanePlaneCollision, checkPlaneOBBCollision},
            {nullptr, nullptr, nullptr, checkOBBOBBCollision}
        };

    public:
        PhysicsWorld() {}
        BodyID createBody(const PhysicsShape& shape, Real mass, PhysicsLayer layer);
        BodyID createBody(const PhysicsShape& shape, const Vector3& position, Real mass, PhysicsLayer layer);
        BodyID createBody(const PhysicsShape& shape, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer);
        BodyID createBody(const PhysicsShape& shape, const PhysicsMaterial& material, Real mass, PhysicsLayer layer);
        BodyID createBody(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, Real mass, PhysicsLayer layer);
        BodyID createBody(const PhysicsShape& shape, const PhysicsMaterial& material, const Vector3& position, const Quaternion& orientation, Real mass, PhysicsLayer layer);
        // void remove(BodyID id);

        // Body manipulation functions
        void setLinearVelocity(BodyID id, const Vector3& v);
        void setAngularVelocity(BodyID id, const Vector3& omega);
        
        Matrix4 getWorldMatrix(BodyID id);

        // This should be outside of this class but for now it's ok
        bool isColliding(BodyID a, BodyID b);

        void setGravity(const Vector6& grav);

        // void set_time_step(Real duration);

        // TODO: Updates with 1 / 60 second granularity. If delta > 1 / 60 the integration step is done multiple times
        void update(Real delta);  // This is where the integration actually occurs
};