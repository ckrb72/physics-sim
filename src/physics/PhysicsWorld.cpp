#include "physics.h"
#include <util/util.h>

PhysicsWorld::PhysicsWorld()
{

}

int32_t PhysicsWorld::create_body(std::shared_ptr<PhysicsShape> shape, double mass, PhysicsLayer layer)
{
    int32_t id = bodies.size();
    bodies.push_back(PhysicsBody(shape, mass, layer));
    return id;
}

int32_t PhysicsWorld::create_body(std::shared_ptr<PhysicsShape> shape, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double mass, PhysicsLayer layer)
{
    int32_t id = bodies.size();
    bodies.push_back(PhysicsBody(shape, position, orientation, mass, layer));
    return id;
}

int32_t PhysicsWorld::create_body(std::shared_ptr<PhysicsShape> shape, const PhysicsMaterial& material, double mass, PhysicsLayer layer)
{
    int32_t id = bodies.size();
    bodies.push_back(PhysicsBody(shape, material, mass, layer));
    return id;
}

int32_t PhysicsWorld::create_body(std::shared_ptr<PhysicsShape> shape, const PhysicsMaterial& material, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double mass, PhysicsLayer layer)
{
    int32_t id = bodies.size();
    bodies.push_back(PhysicsBody(shape, material, position, orientation, mass, layer));
    return id;
}

Eigen::Matrix4d PhysicsWorld::get_world_matrix(int32_t id)
{
    if (id < 0 || id > bodies.size() - 1) return {};

    return bodies[id].get_world_matrix();
}

void PhysicsWorld::set_linear_velocity(int32_t id, const Eigen::Vector3d& v)
{
    if (id < 0 || id > bodies.size() - 1) return;

    bodies[id].set_linear_velocity(v);
}

void PhysicsWorld::set_angular_velocity(int32_t id, const Eigen::Vector3d& omega)
{
    if (id < 0 || id > bodies.size() - 1) return;
    bodies[id].set_angular_velocity(omega);
}

BodyInfo PhysicsWorld::get_info(int32_t id) const
{
    if (id < 0 || id > bodies.size()) return {};
    const PhysicsBody& body = bodies[id];
    return BodyInfo{
        .mass = body.mass,
        .position = body.transform.position,
        .orientation = body.transform.orientation,
        .linear_momentum = body.linear_momentum,
        .angular_momentum = body.angular_momentum,
        .force = body.force,
        .impulse = body.impulse,
        .torque = body.torque,
        .torque_impulse = body.torque_impulse
    };
}

CollisionResult PhysicsWorld::check_collision(PhysicsBody* a, PhysicsBody* b)
{
    // Sort by shape type
    if (a->shape->get_type() > b->shape->get_type())
    {
        PhysicsBody* temp = a;
        a = b;
        b = temp;
    }

    // Call correct function depending on a and b's types
    return collision_funcs[a->shape->get_type()][b->shape->get_type()](&(*a->shape), &a->transform, &(*b->shape), &b->transform); 
}

CollisionResult PhysicsWorld::check_sphere_sphere_collision(const PhysicsShape* const a, const Transform* const at, const PhysicsShape* const b, const Transform* const bt)
{
    const SphereShape* const a_sphere = (const SphereShape* const)a;
    const SphereShape* const b_sphere = (const SphereShape* const)b;

    Eigen::Vector3d position_diff = bt->position - at->position;

    if ( std::abs(position_diff.norm()) < (a_sphere->r + b_sphere->r))
    {
        return CollisionResult {
            .colliding = true,
            .norm = position_diff
        };
    }
    return CollisionResult { .colliding = false };
}

CollisionResult PhysicsWorld::check_sphere_plane_collision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const plane, const Transform* const plane_transform)
{
    SphereShape* s = (SphereShape*)sphere;
    PlaneShape* p = (PlaneShape*)plane;

    Eigen::Vector3d plane_norm = plane_transform->orientation * Eigen::Vector3d(0.0, 1.0, 0.0);
    plane_norm.normalize();

    // Check collision using mathematical formula
    double norm_projection = plane_norm.dot((sphere_transform->position - plane_transform->position)) / plane_norm.norm();
    
    if (std::abs(norm_projection) < s->r)
    {
        return CollisionResult {
            .colliding = true,
            .norm = plane_norm,
        };
    } 

    return CollisionResult{.colliding = false};
}

CollisionResult PhysicsWorld::check_plane_plane_collision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform)
{
    //NOT_IMPLEMENTED();
    return {};
}

CollisionResult PhysicsWorld::check_sphere_obb_collision(const PhysicsShape* const sphere, const Transform* const sphere_transform, const PhysicsShape* const obb, const Transform* const obb_transform)
{
    NOT_IMPLEMENTED();
    return {};
}

CollisionResult PhysicsWorld::check_plane_obb_collision(const PhysicsShape* const plane, const Transform* const plane_transform, const PhysicsShape* const obb, const Transform* const obb_transform)
{
    NOT_IMPLEMENTED();
    return {};
}


CollisionResult PhysicsWorld::check_obb_obb_collision(const PhysicsShape* const a, const Transform* const a_transform, const PhysicsShape* const b, const Transform* const b_transform)
{
    const OBBShape* const a_shape = (const OBBShape* const)a;
    const OBBShape* const b_shape = (const OBBShape* const)b;

    const float EPSILON = 1e-6f;

    Eigen::Matrix3d a_axis = a_transform->orientation.toRotationMatrix();
    Eigen::Matrix3d b_axis = b_transform->orientation.toRotationMatrix();

    Eigen::Matrix3d rotation;
    Eigen::Matrix3d abs_rotation;

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
    Eigen::Vector3d translation = b_transform->position - a_transform->position;
    translation = Eigen::Vector3d(a_axis.col(0).dot(translation), a_axis.col(1).dot(translation), a_axis.col(2).dot(translation));

    float ra, rb;

    // Test axes of a
    for (int i = 0; i < 3; i++)
    {
        ra = a_shape->half_extent(i);
        rb = b_shape->half_extent.x() * abs_rotation(i, 0) + b_shape->half_extent.y() * abs_rotation(i, 1) + b_shape->half_extent.z() * abs_rotation(i, 2);
        if (std::abs(translation(i)) > ra + rb) return CollisionResult {
            .colliding = false
        };
    }


    // Test axes of b
    for (int i = 0; i < 3; i++)
    {
        ra = a_shape->half_extent.x() * abs_rotation(0, i) + a_shape->half_extent.y() * abs_rotation(1, i) + a_shape->half_extent.z() * abs_rotation(2, i);
        rb = b_shape->half_extent(i);
        if ( std::abs(translation.x() * rotation(0, i) + translation.y() * rotation(1, i) + translation.z() * rotation(2, i)) > ra + rb) return CollisionResult {
            .colliding = false
        };
    }


    // Test cross axes

    ra = a_shape->half_extent.y() * abs_rotation(2, 0) + a_shape->half_extent.z() * abs_rotation(1, 0);
    rb = b_shape->half_extent.y() * abs_rotation(0, 2) + b_shape->half_extent.z() * abs_rotation(0, 1);
    if (std::abs(translation.z() * rotation(1, 0) - translation.y() * rotation(2, 0)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.y() * abs_rotation(2, 1) + a_shape->half_extent.z() * abs_rotation(1, 1);
    rb = b_shape->half_extent.x() * abs_rotation(0, 2) + b_shape->half_extent.z() * abs_rotation(0, 0);
    if (std::abs(translation.z() * rotation(1, 1) - translation.y() * rotation(2, 1)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.y() * abs_rotation(2, 2) + a_shape->half_extent.z() * abs_rotation(1, 2);
    rb = b_shape->half_extent.x() * abs_rotation(0, 1) + b_shape->half_extent.y() * abs_rotation(0, 0);
    if (std::abs(translation.z() * rotation(1, 2) - translation.y() * rotation(2, 2)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(2, 0) + a_shape->half_extent.z() * abs_rotation(0, 0);
    rb = b_shape->half_extent.y() * abs_rotation(1, 2) + b_shape->half_extent.z() * abs_rotation(1, 1);
    if (std::abs(translation.x() * rotation(2, 0) - translation.z() * rotation(0, 0)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(2, 1) + a_shape->half_extent.z() * abs_rotation(0, 1);
    rb = b_shape->half_extent.x() * abs_rotation(1, 2) + b_shape->half_extent.z() * abs_rotation(1, 0);
    if (std::abs(translation.x() * rotation(2, 1) - translation.z() * rotation(0, 1)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(2, 2) + a_shape->half_extent.z() * abs_rotation(0, 2);
    rb = b_shape->half_extent.x() * abs_rotation(1, 1) + b_shape->half_extent.y() * abs_rotation(1, 0);
    if (std::abs(translation.x()* rotation(2, 2) - translation.z() * rotation(0, 2)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(1, 0) + a_shape->half_extent.y() * abs_rotation(0, 0);
    rb = b_shape->half_extent.y() * abs_rotation(2, 2) + b_shape->half_extent.z() * abs_rotation(2, 1);
    if (std::abs(translation.y() * rotation(0, 0) - translation.x() * rotation(1, 0)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(1, 1) + a_shape->half_extent.y() * abs_rotation(0, 1);
    rb = b_shape->half_extent.x() * abs_rotation(2, 2) + b_shape->half_extent.z() * abs_rotation(2, 0);
    if (std::abs(translation.y() * rotation(0, 1) - translation.x() * rotation(1, 1)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    ra = a_shape->half_extent.x() * abs_rotation(1, 2) + a_shape->half_extent.y() * abs_rotation(0, 2);
    rb = b_shape->half_extent.x() * abs_rotation(2, 1) + b_shape->half_extent.y() * abs_rotation(2, 0);
    if (std::abs(translation.y() * rotation(0, 2) - translation.x() * rotation(1, 2)) > ra + rb) return CollisionResult {
        .colliding = false
    };

    return CollisionResult {
        .colliding = true
    };
}

bool PhysicsWorld::is_colliding(int32_t a, int32_t b)
{
    if (a == b) return false;

    if (a > bodies.size() - 1 || b > bodies.size() - 1) return false;

    CollisionResult result = check_collision(&bodies[a], &bodies[b]);
    return result.colliding;
}
       

// TODO: Make it so update runs multiple steps if delta > 1 / 60
void PhysicsWorld::update(double delta)
{
    // Check body collisions and update forces appropriately

    for (int i = 0; i < bodies.size(); i++)
    {
        for (int j = 0; j < bodies.size(); j++)
        {
            if (i == j || (bodies[i].layer == PhysicsLayer::STATIC && bodies[j].layer == PhysicsLayer::STATIC)) continue;

            CollisionResult result = check_collision(&bodies[i], &bodies[j]);
            if (result.colliding) 
            {
                result.norm.normalize();
                Eigen::Vector3d reflected_vec = bodies[i].linear_momentum - 2.0f * bodies[i].linear_momentum.dot(result.norm) * result.norm;
                bodies[i].linear_momentum = reflected_vec * (bodies[i].material.restitution + bodies[j].material.restitution) / 2.0f;
            }

        }
    }


    for (PhysicsBody& body : bodies)
    {
        // Update body position
        if (body.layer == PhysicsLayer::DYNAMIC)
        {
 
            // Gravity = acceleration (-9.8) * mass
            Eigen::Vector3d gravity = grav_acceleration;
            gravity *= body.mass;
            body.add_force(gravity);

            // Move this outside of the physics body.
            // Body shouldn't be responsible for stepping itself
            body.step(delta); 
        }
    }
}

void PhysicsWorld::set_gravity(const Eigen::Vector3d& grav)
{
    this->grav_acceleration = grav;
}





// Computes Force and Torque over time t and places them in rb
// This actually technically calculates the impulse and impulsive torque OVER the time t,
// not the force and torque AT time t.
void compute_force_and_torque(double t, RigidBody& rb)
{
    // Compute force and torque from impulses
    Eigen::Vector3d force = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d torque = Eigen::Vector3d(0.0, 0.0, 0.0);


    /*int impulse_count = impulses.size();

    for (int i = 0; i < impulse_count; i++)
    {
        Impulse& impulse = impulses.front();
        impulses.pop();

        Eigen::Vector3d j = impulse.force;
        double time;

        // Time is either the delta of the frame (t) or the time remaining on the impulse (impulse.time)
        if (t <= impulse.time) time = t;
        else time = impulse.time;

        j *= time;

        force += j;
        torque += glm::cross((impulse.pos - rb.x), j);

        impulse.time -= time;

        if (impulse.time > 0.0) impulses.push(impulse);
    }*/


    // Add constant forces
    // Gravity
    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, 0.0);
    gravity *= t;
    force += gravity;

    Eigen::Vector3d inst_torque = Eigen::Vector3d(0.0, 0.0, 0.0);
    inst_torque *= t;
    torque += inst_torque;
    
    // These are actually technically impulse and impulsive torque since we are multiplying by time
    // But it doesn't really matter as long as we are consistent
    rb.force = force;

    rb.torque = torque;
    rb.torque *= t;
}

// TODO: Have dydt place the force and torque (and other per frame variables) into their own struct and return it (makes no sense to place it in RigidBody if it is per frame)

// Takes a and transforms it into a_star
void vec_to_mat_star(Eigen::Vector3d a, Eigen::Matrix3d& a_star)
{
    //TODO: Check to make sure this is okay
    a_star(0, 0) = 0.0;
    a_star(0, 1) = -a.z();
    a_star(0, 2) = a.y();

    a_star(1, 0) = a.z();
    a_star(1, 1) = 0.0;
    a_star(1, 2) = -a.x();

    a_star(2, 0) = -a.y();
    a_star(2, 1) = a.x();
    a_star(2, 2) = 0.0;
}

// Computes instantaneous changes of rb at time t and places the data into dydt
void dydt(double t, RigidBody& rb)
{
    // Compute Velocity
    rb.v.x() = rb.P.x() / rb.mass;
    rb.v.y() = rb.P.y() / rb.mass;
    rb.v.z() = rb.P.z() / rb.mass;
    rb.v *= t;      // Scale by time
    
    rb.q.normalize();
    rb.R = rb.q.toRotationMatrix();

   
    // Compute inverse Inertia tensor
    rb.Iinv = rb.R * rb.IbodyInv * rb.R.transpose().eval();

    // Compute angular velocity
    rb.omega = rb.Iinv * rb.L;

    compute_force_and_torque(t, rb);

    // Compute qdot (Instantaneous rate of change of orientation encoded in quaternion)
    Eigen::Quaterniond omega_q = Eigen::Quaterniond(0.0f, rb.omega);
    rb.qdot = (omega_q * rb.q);
    rb.qdot.coeffs() *= 0.5;
    rb.qdot.coeffs() *= t;
}

// TODO: add logic to handle the case that delta is too large (split the integration into multiple steps);
void ode(RigidBody& rb, double delta)
{
    // Compute derivatives
    dydt(delta, rb);

    // TOOD: Might want to rename these fields to impulse and impulsive torque since that makes more sense
    // Add force (technically impulse) to linear momentum
    rb.P += rb.force;

    // Add torque (technically impulsive torque) to angular momentum
    rb.L += rb.torque;

    // Compute new position and orientation
    rb.x += rb.v;

    // FIXME: This might be wrong
    rb.q.coeffs() = rb.qdot.coeffs() + (0.5f * rb.q.coeffs());
    rb.q.normalize();
}