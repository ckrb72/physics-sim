#include "render.h"
#include <glad/glad.h>

const std::shared_ptr<Geometry> GeometryFactory::load(const std::vector<Vertex>& vertices, const std::vector<glm::uvec3>& indices)
{
    return std::make_shared<Mesh>(vertices, indices);
}

const std::shared_ptr<Geometry> GeometryFactory::load_sphere(float radius, uint32_t subdivisions)
{
    // Generate base icosphere
    float phi = (1.0f + std::sqrt(5.0f)) * 0.5f;
    float a = 1.0f;
    float b = 1.0f / phi;
    std::vector<Vertex> vertices = 
    {
        { {0.0, b, -a}, {0.0, b, -a} },
        { {b, a, 0.0}, {b, a, 0.0} },
        { {-b, a, 0.0}, {-b, a, 0.0} },
        { {0.0, b, a}, {0.0, b, a} },
        { {0.0, -b, a}, {0.0, -b, a} },
        { {-a, 0.0, b}, {-a, 0.0, b} },
        { {0.0, -b, -a}, {0.0, -b, -a} },
        { {a, 0.0, -b}, {a, 0.0, -b} },
        { {a, 0.0, b}, {a, 0.0, b} },
        { {-a, 0.0, -b}, {-a, 0.0, -b} },
        { {b, -a, 0.0}, {b, -a, 0.0} },
        { {-b, -a, 0.0}, {-b, -a, 0.0} },
    };

    std::vector<glm::uvec3> indices = 
    {
        {0, 1, 2},
        {1, 2, 3},
        {3, 4, 5},
        {3, 8, 4},
        {0, 6, 7},
        {0, 9, 6},
        {4, 10, 11},
        {6, 11, 10},
        {2, 5, 9},
        {11, 9, 5},
        {1, 7, 8}, 
        {10, 8, 7},
        {3, 5, 2}, 
        {3, 1, 8},
        {0, 2, 9},
        {0, 7, 1},
        {6, 9, 11},
        {6, 10, 7},
        {4, 11, 5},
        {4, 8, 10}
    };

    // Subdivide mesh
    for (int i = 0; i < subdivisions; i++)
    {
        
    }

    return load(vertices, indices);
}

const std::shared_ptr<Geometry> GeometryFactory::load_rect(float width, float height, float depth)
{
    float half_width = width / 2.0;
    float half_height = height / 2.0;
    float half_depth = depth / 2.0;

    // Construct cube with given radius
    std::vector<Vertex> vertices = 
    {
        { {-half_width, -half_height, half_depth},  {0.0, 0.0, 1.0} },
        { {half_width, -half_height, half_depth},   {0.0, 0.0, 1.0} },
        { {half_width, half_height, half_depth},    {0.0, 0.0, 1.0} },
        { {-half_width, half_height, half_depth},   {0.0, 0.0, 1.0} },

        { {half_width, -half_height, -half_depth},    {0.0, 0.0, -1.0} },
        { {-half_width, -half_height, -half_depth},   {0.0, 0.0, -1.0} },
        { {-half_width, half_height, -half_depth},    {0.0, 0.0, -1.0} },
        { {half_width, half_height, -half_depth},     {0.0, 0.0, -1.0} },

        { {-half_width, -half_height, -half_depth},   {-1.0, 0.0, 0.0} },
        { {-half_width, -half_height, half_depth},   {-1.0, 0.0, 0.0} },
        { {-half_width, half_height, half_depth},   {-1.0, 0.0, 0.0} },
        { {-half_width, half_height, -half_depth},   {-1.0, 0.0, 0.0} },

        { {half_width, -half_height, half_depth},    {1.0, 0.0, 0.0} },
        { {half_width, -half_height, -half_depth},    {1.0, 0.0, 0.0} },
        { {half_width, half_height, -half_depth},    {1.0, 0.0, 0.0} },
        { {half_width, half_height, half_depth},    {1.0, 0.0, 0.0} },

        { {-half_width, half_height, half_depth},    {0.0, 1.0, 0.0} },
        { {half_width, half_height, half_depth},    {0.0, 1.0, 0.0} },
        { {half_width, half_height, -half_depth},    {0.0, 1.0, 0.0} },
        { {-half_width, half_height, -half_depth},    {0.0, 1.0, 0.0} },

        { {-half_width, -half_height, -half_depth},   {0.0, -1.0, 0.0} },
        { {half_width, -half_height, -half_depth},    {0.0, -1.0, 0.0} },
        { {half_width, -half_height, half_depth},    {0.0, -1.0, 0.0} },
        { {-half_width, -half_height, half_depth},    {0.0, -1.0, 0.} }
    };
    
    std::vector<glm::uvec3> indices = 
    {
        {0, 1, 2},
        {2, 3, 0},

        {4, 5, 6},
        {6, 7, 4},

        {8, 9, 10},
        {10, 11, 8},

        {12, 13, 14},
        {14, 15, 12},

        {16, 17, 18},
        {18, 19, 16},

        {20, 21, 22},
        {22, 23, 20}
    };

    return load(vertices, indices);
}

const std::shared_ptr<Geometry> GeometryFactory::load_plane(float width, float height)
{
    float half_width = width / 2.0;
    float half_height = height / 2.0;

    std::vector<Vertex> vertices = 
    {
        { {-half_width, 0.0, half_height}, {0.0, 1.0, 0.0} },
        { {half_width, 0.0, half_height}, {0.0, 1.0, 0.0} },
        { {half_width, 0.0, -half_height}, {0.0, 1.0, 0.0} },
        { {-half_width, 0.0, -half_height}, {0.0, 1.0, 0.0} }
    };

    std::vector<glm::uvec3> indices = 
    {
        {0, 1, 2},
        {2, 3, 0}
    };

    return load(vertices, indices);
}

Mesh::Mesh(const std::vector<Vertex>& vertices, const std::vector<glm::uvec3>& indices)
{
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, pos));
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, norm));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(glm::uvec3), indices.data(), GL_STATIC_DRAW);

    this->vertices = vertices;
    this->indices = indices;
}

Mesh::~Mesh()
{
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &ebo);
}

void Mesh::draw() const
{
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indices.size() * 3, GL_UNSIGNED_INT, nullptr);
}