#pragma once
#include "physics.h"

struct RigidBodyState
{
    Vector6d position;
    Vector6d velocity;
    Eigen::Matrix<double, 6, 6> spatial_inertia;
};  

Vector6d calculateInverseDynamics(const RigidBodyState& rb, const Vector6d& desiredAcceleration, const Vector6d& externalAcceleration);
Vector6d calculateForwardDynamics(const RigidBodyState& rb, const Vector6d& externalForces);