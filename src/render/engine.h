#pragma once
#include <GLFW/glfw3.h>

class EngineTime
{
    private:
        double current, previous, dt;

    public:
        void init()
        {
            current = glfwGetTime();
            previous = 0.0;
            dt = 0.0;
        };

        void update()
        {
            current = glfwGetTime();
            dt = current - previous;
            previous = current;
        };

        inline double delta() const { return dt; };
};