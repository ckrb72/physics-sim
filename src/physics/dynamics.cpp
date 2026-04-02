#include "dynamics.h"

// will eventually want to pass in a whole kinematic tree with joints but for now just focus on a single body
// Returns the necessary force for the given inputs

static Vector6 forceCrossProduct(const Vector6& a, const Vector6& b)
{

    Vector6 product;
    product << a[1] * b[2] - a[2] * b[1] + a[4] * b[5] - a[5] * b[4],
               a[2] * b[0] - a[0] * b[2] + a[5] * b[3] - a[3] * b[5],
               a[0] * b[1] - a[1] * b[0] + a[3] * b[4] - a[4] * b[3],
               a[1] * b[5] - a[2] * b[4],
               a[2] * b[3] - a[0] * b[5],
               a[0] * b[4] - a[1] * b[3];

    return product;
}

Vector6 calculateInverseDynamics(const RigidBodyState& rb, const Vector6& desiredAcceleration, const Vector6& externalAcceleration)
{

    // For now parent velocity and acceleration are 0
    // There is no need to convert parent coordinates to child coordinates because they will all be in the same coordinate frame for now (there is no parent)

    // Forward Pass: calculate velocity and acceleration of each body from the parent and the join velocity (starts from root)

        // Velocity: 
            // Convert parent velocity to child's coordinate frame

            // Add joint velocity times the subspace that the joint effects

        // Acceleration:
            // Convert parent acceleration to child's coordinate frame
            
            // Add joint acceleration times the subspace that the join effects

            // Add the objects velocity cross product with the joint velocity time the subspcae that the joint effects

        // Inertial Body Forces: (What is this for... just general forces?)
            // Inertia tensor of the body times the body's acceleration

            // Add Body's velocity cross star product with the inertia tensor times the body's velocity

            // Subtract any external forces (expressed in body coordinates)

    // Backward Pass: calculate the body forces required to produce the acceleration (starts from leaf nodes)

        // Parent Force:
            // Convert the force calculated in the forward pass to the coordinate frame of the parent
            
            // Add the converted force to the parent's currently calculated force

            // Project the force along the joint axis
    Vector6 iv = rb.spatial_inertia * rb.velocity;
    Vector6 result = rb.spatial_inertia * (desiredAcceleration - externalAcceleration) + forceCrossProduct(rb.velocity, iv);

    // Won't actually return anything as everything will be stored in the rigid bodies
    return result;
}


Vector6 calculateForwardDynamics(const RigidBodyState& rb, const Vector6& externalForces)
{
    return rb.spatial_inertia.inverse() * (externalForces - forceCrossProduct(rb.velocity, rb.spatial_inertia * rb.velocity));
}

Vector3 getLinearFromSpatial(const Vector6& spatial)
{
    return Vector3(spatial[3], spatial[4], spatial[5]);
}

Vector3 getAngularFromSpatial(const Vector6& spatial)
{
    return Vector3(spatial[0], spatial[1], spatial[2]);
}