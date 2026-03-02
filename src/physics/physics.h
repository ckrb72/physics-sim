#pragma once
#include <vector>
#include <queue>
#include <memory>
#include <Eigen/Dense>
#include <cmath>

inline double DegreesToRadians(double degrees)
{
    return degrees * (M_PI / 180.0);
}

inline std::array<float, 16> EigenMatrixToFloatArray(const Eigen::Matrix4d& mat)
{
    std::array<float, 16> array = {};
    const double* mat_data = mat.data();
    for (int i = 0; i < 16; i++)
    {
        array[i] = static_cast<float>(mat_data[i]);
    }

    return array;
}


struct AABBox
{
    Eigen::Vector3d half_extents;
    Eigen::Vector3d position;
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
    float restitution = 1.0f;
};

// Change this up completely...
// Make these just be a tag essentially that tells what type of shape it is
// Then have a method that gets the data for that specific shape and have the actual
// collision detector do all of the work with the collision stuff by passing the data into a function or something.
class PhysicsShape
{
    protected:
        ShapeType type = ShapeType::SHAPE;

    public:
        virtual Eigen::Matrix3d get_body_mat(double mass) = 0;
        virtual AABBox get_aabb() = 0;
        //virtual std::vector<uint8_t> get_data() = 0;
        inline ShapeType get_type() const { return type; }
};

//Maybe make a distinction between collision shapes and physics shapes???
//Physics shapes would have inertia tensor while collision shape would just have collision detection stuff

// The origin of this sphere is in "body space" so it will always be 0, 0, 0. In the physics body it is attached to the position will change
class SphereShape : public PhysicsShape
{
    public:
        double r;

        SphereShape(double radius);
        Eigen::Matrix3d get_body_mat(double mass) override;
        AABBox get_aabb() override;
};

// The point used for this plane is in "body space" so it will always be 0, 0, 0. In the physics body it is attached to the position will change
class PlaneShape : public PhysicsShape
{
    public:
        Eigen::Vector3d extent;

        PlaneShape(const Eigen::Vector3d& extent);
        Eigen::Matrix3d get_body_mat(double mass) override;
        AABBox get_aabb() override;
};

class OBBShape : public PhysicsShape
{
    public:
        Eigen::Vector3d half_extent;

        OBBShape(const Eigen::Vector3d& half_extent);
        Eigen::Matrix3d get_body_mat(double mass) override;
        AABBox get_aabb() override;
};

/*class ConeShape : public PhysicsShape
{
    private:
        double height;
        double radius;

    public:
        ConeShape(double radius, double height);
        bool collide() override;
        Eigen::Matrix4d get_body_mat() override;
};

class CylinderShape : public PhysicsShape
{
    private:
        double radius;
        double height;
    
    public:
        CylinderShape(double radius, double height);
        bool collide() override;
        Eigen::Matrix4d get_body_mat() override;
};*/


enum PhysicsLayer
{
    DYNAMIC,
    KINEMATIC,
    STATIC
};

struct Transform
{
    Eigen::Vector3d position = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Quaterniond orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
};


class PhysicsBody
{
    private:
        double mass = 0.0;
        Eigen::Matrix3d Ibody = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d IbodyInv = Eigen::Matrix3d::Identity();
        std::shared_ptr<PhysicsShape> shape = nullptr;

        Transform transform;

        Eigen::Vector3d linear_momentum = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d angular_momentum = Eigen::Vector3d(0.0, 0.0, 0.0);

        // Units: N (kgm / s)
        // Multiplied by delta then added to momentum
        Eigen::Vector3d force = Eigen::Vector3d(0.0, 0.0, 0.0);

        // Units: Ns (kgm)
        // Added to momentum directly
        Eigen::Vector3d impulse = Eigen::Vector3d(0.0, 0.0, 0.0);

        // Units: Nm (kgm^2 / s)
        // Multiplied by delta then added to angular momentum
        Eigen::Vector3d torque = Eigen::Vector3d(0.0, 0.0, 0.0);

        // Units: Nms (kgm^2)
        // Added to angular momentum directly
        Eigen::Vector3d torque_impulse = Eigen::Vector3d(0.0, 0.0, 0.0);

        PhysicsLayer layer = PhysicsLayer::STATIC;

        friend class PhysicsWorld;

        PhysicsMaterial material;

    public:
        PhysicsBody() = delete;
        PhysicsBody(std::shared_ptr<PhysicsShape> shape, const PhysicsMaterial& material, double mass, PhysicsLayer layer);
        PhysicsBody(std::shared_ptr<PhysicsShape> shape, const PhysicsMaterial& material, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double mass, PhysicsLayer layer);
        PhysicsBody(std::shared_ptr<PhysicsShape> shape, double mass, PhysicsLayer layer);
        PhysicsBody(std::shared_ptr<PhysicsShape> shape, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double mass, PhysicsLayer layer);
        
        const Eigen::Vector3d& get_position() const;
        const Eigen::Quaterniond& get_orientation() const;
        Eigen::Matrix4d get_world_matrix() const;

        void set_linear_velocity(const Eigen::Vector3d& v);
        void set_angular_velocity(const Eigen::Vector3d& omega);
        void set_position(const Eigen::Vector3d& pos);
        void set_orientation(const Eigen::Quaterniond& orientation);

        void add_force(const Eigen::Vector3d& force);
        void add_force(const Eigen::Vector3d& force, const Eigen::Vector3d& pos);
        void add_torque(const Eigen::Vector3d& torque);

        // Creates an instantaneous change in velocity (either linear or angular depending on the function used)
        void add_impulse(const Eigen::Vector3d& impulse);
        void add_torque_impulse(const Eigen::Vector3d& torque_impulse);

        std::vector<uint8_t> serialize() const;

        void step(double delta);
};

struct BodyInfo
{
    double mass = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Quaterniond orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

    Eigen::Vector3d linear_momentum = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d angular_momentum = Eigen::Vector3d(0.0, 0.0, 0.0);

    Eigen::Vector3d force = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d impulse = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d torque = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d torque_impulse = Eigen::Vector3d(0.0, 0.0, 0.0);
};


struct CollisionResult
{
    bool colliding = false;
    Eigen::Vector3d norm = Eigen::Vector3d(0.0, 0.0, 0.0);
    double depth = 0.0;
};

class PhysicsWorld
{
    private:
        std::vector<PhysicsBody> bodies;
        Eigen::Vector3d grav_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

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
        int32_t create_body(std::shared_ptr<PhysicsShape> shape, double mass, PhysicsLayer layer);
        int32_t create_body(std::shared_ptr<PhysicsShape> shape, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double mass, PhysicsLayer layer);
        int32_t create_body(std::shared_ptr<PhysicsShape> shape, const PhysicsMaterial& material, double mass, PhysicsLayer layer);
        int32_t create_body(std::shared_ptr<PhysicsShape> shape, const PhysicsMaterial& material, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double mass, PhysicsLayer layer);
        void remove(int32_t id);

        // Body manipulation functions
        void set_linear_velocity(int32_t id, const Eigen::Vector3d& v);
        void set_angular_velocity(int32_t id, const Eigen::Vector3d& omega);
        Eigen::Matrix4d get_world_matrix(int32_t id);

        bool is_colliding(int32_t a, int32_t b);

        void set_gravity(const Eigen::Vector3d& grav);

        BodyInfo get_info(int32_t id) const;

        // TODO: Updates with 1 / 60 second granularity. If delta > 1 / 60 the integration step is done multiple times
        void update(double delta);  // This is where the integration actually occurs
};

struct RigidBody
{
    double mass;
    Eigen::Matrix3d Ibody;
    Eigen::Matrix3d IbodyInv;

    Eigen::Vector3d x;    // pos
    Eigen::Quaterniond q;    // orientation expressed as a quaternion
    Eigen::Vector3d P;    // linear momentum
    Eigen::Vector3d L;    // angular momentum

    Eigen::Matrix3d Iinv; // Inverse inertia tensor
    Eigen::Vector3d v;    // velocity
    Eigen::Vector3d omega;// angular velocity
    Eigen::Matrix3d R;    // Orientation derived from quaternion q
    Eigen::Quaterniond qdot; // Change of orientation quaternion

    Eigen::Vector3d force;
    Eigen::Vector3d torque;
};