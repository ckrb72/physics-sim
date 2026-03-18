#pragma once
#include "physics.h"

Vector6d calculateInverseDynamics(const Vector6d& currentPosition, const Vector6d& currentVelocity, const Eigen::Matrix<double, 6, 6>& inertia, const Vector6d& desiredAcceleration, const Vector6d& externalForces);