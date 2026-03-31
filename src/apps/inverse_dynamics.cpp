#include <iostream>
#include <physics/physics.h>
#include <physics/dynamics.h>

int main()
{
    // Inertia tensor for a cube of size (1, 1, 1) with mass 1.0
    const double mass = 1.0;

    // This goes in the top right corner of the spatial_inertia matrix
    // Eigen::Matrix3d inertia_tensor;
    // inertia_tensor << mass / 6.0, 0.0, 0.0,
    //        0.0, mass / 6.0, 0.0,
    //        0.0, 0.0, mass / 6.0;


    Eigen::Matrix<double, 6, 6> spatial_inertia;
    spatial_inertia << mass / 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, mass / 6.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, mass / 6.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, mass, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, mass, 0.0, 
                       0.0, 0.0, 0.0, 0.0, 0.0, mass;

    // Start with no rotation and position at (1, 0, 0)
    Vector6d position;
    position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Start with no velocity
    Vector6d velocity;
    velocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Want a desired rotational acceleration of 1 unit across the x axis
    Vector6d desired_acceleration;
    desired_acceleration << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Gravity is 9.8 m/s2 in the negative y direction
    Vector6d gravity;
    gravity << 0.0, 0.0, 0.0, 0.0, -9.8 * mass, 0.0;

    RigidBodyState state = {
        .position = position,
        .velocity = velocity,
        .spatial_inertia = spatial_inertia
    };

    Vector6d forces = calculateInverseDynamics(state, desired_acceleration, gravity);
    std::cout << forces << std::endl;

    return 0;
}