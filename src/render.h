#pragma once
#include <iostream>
#include <vector>

#include <queue>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

struct Vertex 
{
    glm::vec3 pos;
    glm::vec3 norm;
};

class Geometry
{
    public:
        virtual void draw() const = 0;
};

class GeometryFactory
{
    public:
        static const std::shared_ptr<Geometry> load(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);
        static const std::shared_ptr<Geometry> load_sphere(uint32_t radius, uint32_t resolution);
        static const std::shared_ptr<Geometry> load_cube(uint32_t width, uint32_t height);
};

class Mesh : public Geometry
{
    private:
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;

        unsigned int vao, vbo, ebo;

    public:
        Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);
        ~Mesh();
        Mesh(const Mesh&) = delete;
        Mesh& operator=(const Mesh&) = delete;
        Mesh(Mesh&&) = delete;
        Mesh& operator=(const Mesh&&) = delete;

        void draw() const override;
};


class MeshBatch : public Geometry
{
    private:
        uint32_t offset;
        unsigned int vao, vbo, ebo;

    public:
        MeshBatch(uint32_t size);
        void draw() const override;
        bool push(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);
};

struct Transform
{
    glm::vec3 pos;
    glm::quat orientation;
};


// DISCUSS: Possible idea for renderer architecture
// Physics engine just needs to generate a list of objects that have moved and their new transforms. Each frame simply go through
// that list and update the transforms of the objects that moved. 
class Renderer
{
    private:
        std::vector<Mesh> meshes;
        std::vector<Transform> transforms;
        std::queue<int32_t> free_list;

    public:
        Renderer(uint32_t buf_size);

        int32_t load(const std::vector<Vertex&> vertices, const std::vector<unsigned int>& indices);

        void set_transform(int32_t id);

        void set_visible(int32_t id);

        void remove(int32_t id);

        const std::vector<Vertex>& get_vertices(int32_t id);
};