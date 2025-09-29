#pragma once
#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>

#include <queue>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>


/*
    Should each geometry object hold it's own transform / model matrix or should that be held separately and then tell it to draw it at a given transform?
*/

std::string read_file(const std::string& path);
bool load_shader(const std::string& vertex_path, const std::string& fragment_path, unsigned int* program_ptr);

struct Vertex 
{
    glm::vec3 pos;
    glm::vec3 norm;
};

class Geometry
{
    public:
        virtual void draw(unsigned int shader, const glm::mat4&) const = 0;
};

class GeometryFactory
{
    public:
        static const std::shared_ptr<Geometry> load(const std::vector<Vertex>& vertices, const std::vector<glm::uvec3>& indices);
        static const std::shared_ptr<Geometry> load_sphere(float radius, uint32_t subdivisions);
        static const std::shared_ptr<Geometry> load_rect(float width, float height, float depth);
        static const std::shared_ptr<Geometry> load_plane(float width, float height);
};

class Mesh : public Geometry
{
    private:
        std::vector<Vertex> vertices;
        std::vector<glm::uvec3> indices;

        unsigned int vao, vbo, ebo;

    public:
        Mesh(const std::vector<Vertex>& vertices, const std::vector<glm::uvec3>& indices);
        ~Mesh();
        Mesh(const Mesh&) = delete;
        Mesh& operator=(const Mesh&) = delete;
        Mesh(Mesh&&) = delete;
        Mesh& operator=(const Mesh&&) = delete;

        void draw(unsigned int shader, const glm::mat4& model) const override;
};


class MeshBatch : public Geometry
{
    private:
        uint32_t offset;
        unsigned int vao, vbo, ebo;
        unsigned int shader;

    public:
        MeshBatch(uint32_t size);
        void draw(unsigned int shader, const glm::mat4& model) const override;
        bool push(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);
};

// DISCUSS: Possible idea for renderer architecture
// Physics engine just needs to generate a list of objects that have moved and their new transforms. Each frame simply go through
// that list and update the transforms of the objects that moved. 
/*class Renderer
{
    private:
        // Indexed by id
        std::vector<Mesh> meshes;
        std::vector<Transform> transforms;
        std::vector<glm::mat4> world_matrices;

        // Holds available ids
        std::queue<int32_t> free_list;

        // Mapping of ids to their children
        std::unordered_map<int32_t, std::vector<int32_t>> hierarchies;

        // Render State Information
        // Probably want to sort / batch / break up draws by shader id
        unsigned int bound_shader;

    public:
        Renderer(uint32_t buf_size);

        int32_t load(const std::vector<Vertex&> vertices, const std::vector<unsigned int>& indices);
        int32_t load(const std::string& path);
        
        void set_transform(int32_t id);

        void set_visible(int32_t id);

        void remove(int32_t id);

        const std::vector<Vertex>& get_vertices(int32_t id);

        void reparent(int32_t id, int32_t parent);
};*/