#include "physics.h"
#include "util.h"

PhysicsWorld::PhysicsWorld()
{

}

int32_t PhysicsWorld::create_body(std::shared_ptr<PhysicsShape> shape, double mass, PhysicsLayer layer)
{
    int32_t id = bodies.size();
    bodies.push_back(PhysicsBody(shape, mass, layer));
    return id;
}

int32_t PhysicsWorld::create_body(std::shared_ptr<PhysicsShape> shape, const glm::vec3& position, const glm::quat& orientation, double mass, PhysicsLayer layer)
{
    int32_t id = bodies.size();
    bodies.push_back(PhysicsBody(shape, position, orientation, mass, layer));
    return id;
}

glm::mat4 PhysicsWorld::get_world_matrix(int32_t id)
{
    if (id < 0 || id > bodies.size() - 1) return {};

    return bodies[id].get_world_matrix();
}

void PhysicsWorld::set_linear_velocity(int32_t id, const glm::vec3& v)
{
    if (id < 0 || id > bodies.size() - 1) return;

    bodies[id].set_linear_velocity(v);
}

void PhysicsWorld::set_global_force(const glm::vec3& force)
{
    global_force = force;
}

BodyInfo PhysicsWorld::get_info(int32_t id) const
{
    if (id < 0 || id > bodies.size()) return {};
    const PhysicsBody& body = bodies[id];
    return BodyInfo{
        .mass = body.mass,
        .position = body.position,
        .orientation = body.orientation,
        .linear_momentum = body.linear_momentum,
        .angular_momentum = body.angular_momentum,
        .force = body.force,
        .impulse = body.impulse,
        .torque = body.torque,
        .torque_impulse = body.torque_impulse
    };
}

CollisionResult PhysicsWorld::check_collision(PhysicsBody& a, PhysicsBody& b)
{
    // Sort by shape type
    /*if (a.shape->get_type() > b.shape->get_type())
    {
        std::swap(a, b);
    }*/

    // Call correct function depending on a and b's types
    //return collision_func[a.shape->get_type()][b.shape->get_type()](a, b); 

    return CollisionResult{};
}

// TODO: Make it so update runs multiple steps if delta > 1 / 60
void PhysicsWorld::update(double delta)
{
    // Check body collisions and update forces appropriately

    for (int i = 0; i < bodies.size(); i++)
    {
        for (int j = 0; j < bodies.size(); j++)
        {
            if (i == j) continue;

            // Do collision detection here

            //CollisionResult result = check_collision(bodies[i], bodies[j]);

        }
    }


    for (PhysicsBody& body : bodies)
    {
        // Update body position
        if (body.layer == PhysicsLayer::DYNAMIC)
        {
            // Add forces (impulses instead?) as needed (if two objects collide, constant forces, etc.)
            // FIXME: This should not be done every frame. The forces on an object should not be set every frame???
            // Impulses should be able to be added to an object with a vector given in N*s that should not be multiplied by delta
            body.add_impulse(global_force);

            // Move this outside of the physics body.
            // Body shouldn't be responsible for stepping itself
            body.step(delta); 
        }
    }
}





// Computes Force and Torque over time t and places them in rb
// This actually technically calculates the impulse and impulsive torque OVER the time t,
// not the force and torque AT time t.
void compute_force_and_torque(double t, RigidBody& rb)
{
    // Compute force and torque from impulses
    glm::vec3 force = glm::vec3(0.0);
    glm::vec3 torque = glm::vec3(0.0);


    /*int impulse_count = impulses.size();

    for (int i = 0; i < impulse_count; i++)
    {
        Impulse& impulse = impulses.front();
        impulses.pop();

        glm::vec3 j = impulse.force;
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
    glm::vec3 gravity = glm::vec3(0.0, 0.0, 0.0);
    gravity *= t;
    force += gravity;

    glm::vec3 inst_torque = glm::vec3(0.0, 0.0, 0.0);
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
void vec_to_mat_star(glm::vec3 a, glm::mat3& a_star)
{
    //TODO: Check to make sure this is okay
    a_star[0][0] = 0.0;
    a_star[0][1] = -a.z;
    a_star[0][2] = a.y;

    a_star[1][0] = a.z;
    a_star[1][1] = 0.0;
    a_star[1][2] = -a.x;

    a_star[2][0] = -a.y;
    a_star[2][1] = a.x;
    a_star[2][2] = 0.0;
}

// Computes instantaneous changes of rb at time t and places the data into dydt
void dydt(double t, RigidBody& rb)
{
    // Compute Velocity
    rb.v[0] = rb.P[0] / rb.mass;
    rb.v[1] = rb.P[1] / rb.mass;
    rb.v[2] = rb.P[2] / rb.mass;
    rb.v *= t;      // Scale by time
    
    rb.q = glm::normalize(rb.q);
    rb.R = glm::toMat3(rb.q);

   
    // Compute inverse Inertia tensor
    rb.Iinv = rb.R * rb.IbodyInv * glm::transpose(rb.R);

    // Compute angular velocity
    rb.omega = rb.Iinv * rb.L;

    compute_force_and_torque(t, rb);

    // Compute qdot (Instantaneous rate of change of orientation encoded in quaternion)
    glm::quat omega_q = glm::quat(0.0f, rb.omega);
    rb.qdot = (omega_q * rb.q);
    rb.qdot *= 0.5;
    rb.qdot *= t;
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
    rb.q = glm::normalize(rb.qdot + (0.5f * rb.q));
}