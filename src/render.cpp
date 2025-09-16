#include "render.h"
#include <glad/glad.h>

static glm::vec3 slerp(const glm::vec3& a, const glm::vec3& b, double t)
{
    float theta = acos(glm::dot(a, b));
    return glm::normalize(( (float)((sin(1.0 - t) * theta) / sin(theta)) * a ) + ( (float)(sin(t * theta) / sin(theta)) * b ));
}

const std::shared_ptr<Geometry> GeometryFactory::load(const std::vector<Vertex>& vertices, const std::vector<glm::uvec3>& indices)
{
    return std::make_shared<Mesh>(vertices, indices);
}

const std::shared_ptr<Geometry> GeometryFactory::load_sphere(float radius, uint32_t subdivisions)
{
    // Generate base icosphere
    float phi = (1.0f + std::sqrt(5.0f)) * 0.5f;

    float a = std::sqrt(1.0 / (1 + (phi * phi)));
    float c = a * phi;

    std::vector<Vertex> vertices = 
    {
        { {-c, -a, 0.0}, {-c, -a, 0.0} },
        { {c, -a, 0.0}, {c, -a, 0.0} },
        { {c, a, 0.0}, {c, a, 0.0} },
        { {-c, a, 0.0}, {-c, a, 0.0} },

        { {a, 0.0, c}, {a, 0.0, c} },
        { {a, 0.0, -c}, {a, 0.0, -c} },
        { {-a, 0.0, -c}, {-a, 0.0, -c} },
        { {-a, 0.0, c}, {-a, 0.0, c} },

        { {0.0, c, a}, {0.0, c, a} },
        { {0.0, -c, a}, {0.0, -c, a} },
        { {0.0, -c, -a}, {0.0, -c, -a} },
        { {0.0, c, -a}, {0.0, c, -a} }
    };

    std::vector<glm::uvec3> indices = 
    {
        {0, 7, 3},
        {0, 9, 7},
        {7, 8, 3},
        {7, 4, 8},
        {7, 9, 4},
        {9, 1, 4},
        {4, 1, 2},
        {4, 2, 8},
        {2, 11, 8},
        {8, 11, 3},
        {9, 10, 1},
        {10, 9, 0},
        {1, 10, 5},
        {1, 5, 2},
        {2, 5, 11},
        {5, 6, 11},
        {6, 3, 11},
        {5, 6, 10},
        {0, 3, 6},
        {10, 0, 6}
    };

    // Subdivide mesh
    for (int i = 0; i < subdivisions; i++)
    {
        std::vector<Vertex> new_vertices;
        std::vector<glm::uvec3> new_indices;

        uint32_t index_offset = 0;

        for (glm::uvec3 tri : indices)
        {
            glm::vec3 reference_dir = glm::vec3(0.0f, 0.0f, 1.0f);
            glm::vec3 a = vertices[tri[0]].pos;
            glm::vec3 b = vertices[tri[1]].pos;
            glm::vec3 c = vertices[tri[2]].pos;
 
            
            glm::vec3 ab = slerp(a, b, 0.5f);
            glm::vec3 bc = slerp(b, c, 0.5f);
            glm::vec3 ca = slerp(c, a, 0.5f);

            new_vertices.push_back({ a, a });
            new_vertices.push_back({ ab, ab}); 
            new_vertices.push_back({ b, b });
            new_vertices.push_back({ bc, bc });
            new_vertices.push_back({ c, c });
            new_vertices.push_back({ ca, ca });

            new_indices.push_back({index_offset, index_offset + 1, index_offset + 5});
            new_indices.push_back({index_offset + 1, index_offset + 2, index_offset + 3});
            new_indices.push_back({index_offset + 1, index_offset + 3, index_offset + 5});
            new_indices.push_back({index_offset + 5, index_offset + 3, index_offset + 4});
            index_offset += 6;
       }

        vertices = new_vertices;
        indices = new_indices;
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

void Mesh::draw(unsigned int shader, const glm::mat4& model) const
{
    glUseProgram(shader);
    glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm::value_ptr(model));

    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indices.size() * 3, GL_UNSIGNED_INT, nullptr);
}