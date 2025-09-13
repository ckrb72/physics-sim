#pragma once
#include "physics.h"
#include "render.h"


class Simulator
{
    private:
        PhysicsEngine physics;
        Renderer renderer;

    public:
        void run();
};