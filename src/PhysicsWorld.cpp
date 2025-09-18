#include "physics.h"
#include "util.h"

PhysicsWorld::PhysicsWorld()
{

}

int32_t PhysicsWorld::create_body(std::shared_ptr<PhysicsShape> shape, double mass)
{
    int32_t id = bodies.size();
    bodies.push_back(PhysicsBody(shape, mass));
    return id;
}

int32_t PhysicsWorld::create_body(std::shared_ptr<PhysicsShape> shape, const glm::vec3& position, const glm::quat& orientation, double mass)
{
    int32_t id = bodies.size();
    bodies.push_back(PhysicsBody(shape, position, orientation, mass));
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


// TODO: Make it so update runs multiple steps if delta > 1 / 60
void PhysicsWorld::update(double delta)
{
    // Check body collisions and update forces appropriately


    for (PhysicsBody& body : bodies)
    {

        // Update body position

        // Add forces as needed (if two objects collide, constant forces, etc.)
        body.add_force(glm::vec3(0.0, -9.8, 0.0));
        body.step(delta);
    }
}