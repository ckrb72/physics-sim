#pragma once
#include "physics.h"

struct RigidBodyState
{
    Vector6 velocity;
    Eigen::Matrix<Real, 6, 6> spatial_inertia;
};  

Vector6 calculateInverseDynamics(const RigidBodyState& rb, const Vector6& desiredAcceleration, const Vector6& externalAcceleration);
Vector6 calculateForwardDynamics(const RigidBodyState& rb, const Vector6& externalForces);

Vector3 getLinearFromSpatial(const Vector6& spatial);
Vector3 getAngularFromSpatial(const Vector6& spatial);
