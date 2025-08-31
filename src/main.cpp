#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

struct RigidBody
{
    double mass;
    glm::mat3 Ibody;
    glm::mat3 IbodyInv;

    glm::vec3 x;    // pos
    glm::quat q;    // orientation expressed as a quaternion
    glm::vec3 P;    // linear momentum
    glm::vec3 L;    // angular momentum

    glm::mat3 Iinv; // Inverse inertia tensor
    glm::vec3 v;    // velocity
    glm::vec3 omega;// angular velocity
    glm::mat3 R;    // Orientation derived from quaternion q
    glm::quat qdot; // Change of orientation quaternion

    glm::vec3 force;
    glm::vec3 torque;
};


bool load_shader(const std::string& vertex_path, const std::string& fragment_path, unsigned int* program_ptr);

void glfw_error_fun(int error, const char* err_desc)
{
    std::cout << err_desc << std::endl;
}

const int WIN_WIDTH = 1920;
const int WIN_HEIGHT = 1080;

// Computes Force and Torque at time t and places them in rb
void compute_force_and_torque(double t, RigidBody& rb)
{
    // Just set rb to a constant force right now
    rb.force = glm::vec3(0.0, -9.8, 0.0);
    rb.torque = glm::vec3(0.0);
}

// TODO: Have dydt place the force and torque (and other per frame variables) into their own struct and return it (makes no sense to place it in RigidBody if it is per frame)

// Takes a and transforms it into a_star
void vec_to_mat_star(glm::vec3 a, glm::mat3& a_star)
{
    //TODO: Check to make sure this is okay
    a_star[0][0] = 0.0;
    a_star[0][1] = -a.z;
    a_star[0][2] = a.y;

    a_star[1][0] = a.z;
    a_star[1][1] = 0.0;
    a_star[1][2] = -a.x;

    a_star[2][0] = -a.y;
    a_star[2][1] = a.x;
    a_star[2][2] = 0.0;
}

// Computes instantaneous changes of rb at time t and places the data into dydt
void dydt(double t, RigidBody& rb)
{
    // Compute Velocity
    rb.v[0] = rb.P[0] / rb.mass;
    rb.v[1] = rb.P[1] / rb.mass;
    rb.v[2] = rb.P[2] / rb.mass;
    
    rb.R = glm::toMat3(glm::normalize(rb.q));

    // Compute inverse Inertia tensor
    rb.Iinv = rb.R * rb.IbodyInv * glm::transpose(rb.R);

    // Compute angular velocity
    rb.omega = rb.Iinv * rb.L;

    // Right now the torque, velocity, etc are placed into rb itself but really should put these in their own struct because they are per frame
    compute_force_and_torque(t, rb);

    // Compute qdot Instantaneous rate of change of orientation encoded in quaternion)
    rb.qdot = (glm::quat((double)0.0, rb.omega) * rb.q);
    rb.qdot *= 0.5;
}

void ode(RigidBody& rb, double start, double end)
{
    dydt(end, rb);
}

int main()
{
    glfwInit();
    glfwSetErrorCallback(glfw_error_fun);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "Physics Sim", nullptr, nullptr);

    if(!window)
    {
        std::cerr << "Failed to create window" << std::endl;
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);


    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to load proc addresses" << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        exit(EXIT_FAILURE);
    }


    glClearColor(0.3, 0.3, 0.3, 1.0);
    unsigned int vao, vbo, ebo;


    float vertices[] = 
    {
        -0.5, -0.5, 0.5,    0.0, 0.0, 1.0,
        0.5, -0.5, 0.5,     0.0, 0.0, 1.0,
        0.5, 0.5, 0.5,      0.0, 0.0, 1.0,
        -0.5, 0.5, 0.5,     0.0, 0.0, 1.0,

        0.5, -0.5, -0.5,    0.0, 0.0, -1.0,
        -0.5, -0.5, -0.5,   0.0, 0.0, -1.0,
        -0.5, 0.5, -0.5,    0.0, 0.0, -1.0,
        0.5, 0.5, -0.5,     0.0, 0.0, -1.0,

        -0.5, -0.5, -0.5,   -1.0, 0.0, 0.0,
        -0.5, -0.5, 0.5,    -1.0, 0.0, 0.0,
        -0.5, 0.5, 0.5,     -1.0, 0.0, 0.0,
        -0.5, 0.5, -0.5,    -1.0, 0.0, 0.0,

        0.5, -0.5, 0.5,     1.0, 0.0, 0.0,
        0.5, -0.5, -0.5,    1.0, 0.0, 0.0,
        0.5, 0.5, -0.5,     1.0, 0.0, 0.0,
        0.5, 0.5, 0.5,      1.0, 0.0, 0.0,

        -0.5, 0.5, 0.5,     0.0, 1.0, 0.0,
        0.5, 0.5, 0.5,      0.0, 1.0, 0.0,
        0.5, 0.5, -0.5,     0.0, 1.0, 0.0,
        -0.5, 0.5, -0.5,    0.0, 1.0, 0.0,

        -0.5, -0.5, -0.5,   0.0, -1.0, 0.0,
        0.5, -0.5, -0.5,    0.0, -1.0, 0.0,
        0.5, -0.5, 0.5,     0.0, -1.0, 0.0,
        -0.5, -0.5, 0.5,    0.0, -1.0, 0.0
    };

    unsigned int indices[] = 
    {
        0, 1, 2,
        2, 3, 0,

        4, 5, 6,
        6, 7, 4,

        8, 9, 10,
        10, 11, 8,

        12, 13, 14,
        14, 15, 12,

        16, 17, 18,
        18, 19, 16,

        20, 21, 22,
        22, 23, 20
    };


    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &ebo);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    unsigned int program;

    if(!load_shader("../shader/default.vert", "../shader/default.frag", &program))
    {
        std::cout << "Failed to load shader" << std::endl;

        exit(EXIT_FAILURE);
    }

    glm::mat4 perspective = glm::perspective(glm::radians(90.0f), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 10.0f);

    glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 2.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0));
    
    glm::mat4 model = glm::rotate(glm::mat4(1.0f), glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));


    glUseProgram(program);
    glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, glm::value_ptr(perspective));
    glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(model));
 
    std::cout << "Starting simulation" << std::endl;

    double mass = 10.0;
    glm::vec3 dimensions = {1.0, 1.0, 1.0};

    RigidBody rb = 
    {
        .mass = mass,
        .Ibody = {
            {mass / 12.0 * ( (dimensions[1] * dimensions[1]) + (dimensions[2] * dimensions[2]) ), 0.0, 0.0},
            {0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[2] * dimensions[2]) ), 0.0},
            {0.0, 0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[1] * dimensions[1]) )}
        },
        .x = glm::vec3(0.0),    // Position = origin
        .q = glm::quat(0.0, glm::vec3(1.0, 0.0, 0.0)), // Orientation = 0 degree around x axis
        .P = glm::vec3(1.0,0.0, 0.0),   // Linear momentum
        .L = glm::vec3(0.0)             // No angular momentum
    };
    
    glEnable(GL_DEPTH_TEST);
    double previous_time = glfwGetTime();
    double elapsed_time = 0.0;
    float angle = 0.0f;
    while(!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        double current_time = glfwGetTime();
        double delta = current_time - previous_time;
        previous_time = current_time;

        elapsed_time += delta;
        if (elapsed_time > 0.5)
        {
            // Run simulation
            std::cout << "Running simulation step" << std::endl;


            angle += 10.0f;
            glm::mat4 model = glm::rotate(glm::mat4(1.0f), glm::radians(angle), glm::vec3(1.0f, 0.0f, 0.0f));
            glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(model));

            // The time variables are almost certainly wrong but for now this is fine
            //ode(rb, current_time, current_time + delta);

            // Display objects
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glUseProgram(program);
            glBindVertexArray(vao);
            glDrawElements(GL_TRIANGLES, sizeof(indices) / sizeof(unsigned int), GL_UNSIGNED_INT, nullptr);

            glfwSwapBuffers(window);
 

            elapsed_time = 0.0;
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}


std::string read_file(const std::string& path)
{
    std::ifstream file;
    file.open(path, std::ios::in);

    if(!file.is_open())
    {
        std::cout << "failed to open file" << std::endl;

        return "";
    }

    std::string str;
    std::string line;

    while(std::getline(file, line))
    {
        str += line + "\n";
    }

    file.close();

    return str;
}

bool load_shader(const std::string& vertex_path, const std::string& fragment_path, unsigned int* program_ptr)
{
    const int MAX_LOG = 512;
    int success;
    char log[MAX_LOG];

    std::string vertex_str = read_file(vertex_path);

    const char* const vertex_src = vertex_str.c_str();

    unsigned int vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_src, nullptr);
    glCompileShader(vertex_shader);

    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        glGetShaderInfoLog(vertex_shader, MAX_LOG, nullptr, log);
        std::cerr << "Vertex: " << std::endl;
        std::cerr << log << std::endl;
        glDeleteShader(vertex_shader);
        return false;
    }

    std::string fragment_str = read_file(fragment_path);

    const char* const fragment_src = fragment_str.c_str();

    unsigned int fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_src, nullptr);
    glCompileShader(fragment_shader);

    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        glGetShaderInfoLog(fragment_shader, MAX_LOG, nullptr, log);
        std::cerr << "Fragment: " << std::endl;
        std::cerr << log << std::endl;
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        return false;
    }

    unsigned int program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if(!success)
    {
        glGetProgramInfoLog(program, MAX_LOG, nullptr, log);
        std::cerr << "Linker: " << std::endl;
        std::cerr << log << std::endl;
        glDetachShader(program, vertex_shader);
        glDetachShader(program, fragment_shader);
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        glDeleteProgram(program);
        return false;
    }

    glDetachShader(program, vertex_shader);
    glDetachShader(program, fragment_shader);

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    *program_ptr = program;

    return true;
}