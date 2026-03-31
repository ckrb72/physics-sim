#include <iostream>
#include <physics/physics.h>
#include <physics/dynamics.h>

int main()
{

    const double mass = 1.0;

    Eigen::Matrix<double, 6, 6> spatial_inertia;
    spatial_inertia << mass / 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, mass / 6.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, mass / 6.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, mass, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, mass, 0.0, 
                       0.0, 0.0, 0.0, 0.0, 0.0, mass;

    Vector6d velocity;
    velocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    Vector6d gravity;
    gravity << 1.0, 0.0, 0.0, 0.0, -9.8 * mass, 0.0;

    RigidBodyState state = {
        .position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        .velocity = velocity,
        .spatial_inertia = spatial_inertia
    };

    // In the simulation would multiply this by the delta time of the frame or the integration step
    Vector6d acceleration = calculateForwardDynamics(state, gravity);
    std::cout << acceleration << std::endl;
    return 0;
}