#pragma once

class Camera
{
    private:
        glm::mat4 projection = glm::mat4(1.0f);
        glm::mat4 view = glm::mat4(1.0f);

        glm::vec3 position = glm::vec3(0.0f);
        glm::vec3 look = glm::vec3(0.0f);
        bool dirty = false;

    public:
        Camera(const glm::vec3& position, const glm::vec3& look);

        void set_position(const glm::vec3& pos);
        void set_look(const glm::vec3& look);
        void set_perspective(float fov, float aspect_ratio, float near, float far);
        inline void get_projection() const { return projection; }
        void get_view();
};