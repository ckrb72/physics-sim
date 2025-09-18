#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <string>
#include <queue>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>


#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

#include "render.h"
#include "physics.h"

/*

    Questions:
        I'm technically thinking of things in terms of impulse and impulsive torque rather than forces and torques. This makes it easier to work with momentum
        since you just add impulse to momentum. The big implication of this is that my engine computes things based on time deltas rather than computing the force at an arbitrary given time.
        Will this screw me over in the long run or is this okay?

        How should I be architecting this? Should impulses be their own objects that you can assign to rigid bodies or should they be something else?

*/

// This is an example of an impulse queue for a specific rigid body.
// When we add multiple bodies will want to have one of these for each body.
// Might want to put these in a map so can easily add impulses to different bodies (std::map<uint32_t, std::queue<Impulse>>)
// Then each frame every rigid body goes through its queue and dequeues the impulse, computes the force and torque then subtracts the delta time
// from impulse.time. If there is still time remaining, place it back in the queue for next frame.
std::queue<Impulse> impulses;

const int MAX_AABB = 10;
const int AABB_VERT_COUNT = 8;

double cam_radius = 5.0;

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    cam_radius -= 0.2 * yoffset;
}

bool load_shader(const std::string& vertex_path, const std::string& fragment_path, unsigned int* program_ptr);

void glfw_error_fun(int error, const char* err_desc)
{
    std::cout << err_desc << std::endl;
}

const int WIN_WIDTH = 1920;
const int WIN_HEIGHT = 1080;

// Computes Force and Torque over time t and places them in rb
// This actually technically calculates the impulse and impulsive torque OVER the time t,
// not the force and torque AT time t.
void compute_force_and_torque(double t, RigidBody& rb)
{
    // Compute force and torque from impulses
    glm::vec3 force = glm::vec3(0.0);
    glm::vec3 torque = glm::vec3(0.0);


    int impulse_count = impulses.size();

    for (int i = 0; i < impulse_count; i++)
    {
        Impulse& impulse = impulses.front();
        impulses.pop();

        glm::vec3 j = impulse.force;
        double time;

        // Time is either the delta of the frame (t) or the time remaining on the impulse (impulse.time)
        if (t <= impulse.time) time = t;
        else time = impulse.time;

        j *= time;

        force += j;
        torque += glm::cross((impulse.pos - rb.x), j);

        impulse.time -= time;

        if (impulse.time > 0.0) impulses.push(impulse);
    }


    // Add constant forces
    // Gravity
    glm::vec3 gravity = glm::vec3(0.0, 0.0, 0.0);
    gravity *= t;
    force += gravity;

    glm::vec3 inst_torque = glm::vec3(0.0, 0.0, 0.0);
    inst_torque *= t;
    torque += inst_torque;
    
    // These are actually technically impulse and impulsive torque since we are multiplying by time
    // But it doesn't really matter as long as we are consistent
    rb.force = force;

    rb.torque = torque;
    rb.torque *= t;
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
    rb.v *= t;      // Scale by time
    
    rb.q = glm::normalize(rb.q);
    rb.R = glm::toMat3(rb.q);

   
    // Compute inverse Inertia tensor
    rb.Iinv = rb.R * rb.IbodyInv * glm::transpose(rb.R);

    // Compute angular velocity
    rb.omega = rb.Iinv * rb.L;

    compute_force_and_torque(t, rb);

    // Compute qdot (Instantaneous rate of change of orientation encoded in quaternion)
    glm::quat omega_q = glm::quat(0.0f, rb.omega);
    rb.qdot = (omega_q * rb.q);
    rb.qdot *= 0.5;
    rb.qdot *= t;
}

// TODO: add logic to handle the case that delta is too large (split the integration into multiple steps);
void ode(RigidBody& rb, double delta)
{
    // Compute derivatives
    dydt(delta, rb);

    // TOOD: Might want to rename these fields to impulse and impulsive torque since that makes more sense
    // Add force (technically impulse) to linear momentum
    rb.P += rb.force;

    // Add torque (technically impulsive torque) to angular momentum
    rb.L += rb.torque;

    // Compute new position and orientation
    rb.x += rb.v;

    // FIXME: This might be wrong
    rb.q = glm::normalize(rb.qdot + (0.5f * rb.q));
}


void draw_ui(RigidBody& rb);

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
    glfwSetScrollCallback(window, scroll_callback);


    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to load proc addresses" << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        exit(EXIT_FAILURE);
    }


    glClearColor(0.3, 0.3, 0.3, 1.0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    std::shared_ptr<Geometry> sphere = GeometryFactory::load_sphere(1.0, 3);
    std::shared_ptr<Geometry> plane = GeometryFactory::load_plane(10.0, 10.0);
    std::shared_ptr<Geometry> right_plane = GeometryFactory::load_plane(10.0, 10.0);

    PhysicsWorld world;
    int32_t id = world.create_body(std::make_shared<BoxShape>(glm::vec3(1.0f)), 100.0);

    glfwSwapInterval(0);

    unsigned int program;
    if(!load_shader("../shader/default.vert", "../shader/default.frag", &program))
    {
        std::cout << "Failed to load shader" << std::endl;

        exit(EXIT_FAILURE);
    }

    glm::mat4 perspective = glm::perspective(glm::radians(90.0f), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 50.0f);
    glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0));
    
    glUseProgram(program);
    glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, glm::value_ptr(perspective));
    glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view));

 
    const double mass = 10.0;
    glm::vec3 dimensions = {1.0, 1.0, 1.0};

    /*const glm::mat3 Ibody = 
    {
        {mass / 12.0 * ( (dimensions[1] * dimensions[1]) + (dimensions[2] * dimensions[2]) ), 0.0, 0.0},
        {0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[2] * dimensions[2]) ), 0.0},
        {0.0, 0.0, mass / 12.0 * ( (dimensions[0] * dimensions[0]) + (dimensions[1] * dimensions[1]) )}
    }; */

    const double radius = 1.0;

    const double inertia_scale = (2.0 / 5.0) * mass * radius * radius;

    const glm::mat3 Ibody = 
    {
        { inertia_scale, 0.0, 0.0 },
        { 0.0, inertia_scale, 0.0 },
        { 0.0, 0.0, inertia_scale }
    };

    RigidBody rb = 
    {
        .mass = mass,
        .Ibody = Ibody,
        .IbodyInv = glm::inverse(Ibody),
        .x = glm::vec3(0.0, 0.0, 0.0),    // Position = origin
        .q = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.0, 0.0)), // Orientation = 0 degree around x axis
        .P = glm::vec3(0.0, 0.0, 0.0),   // Linear momentum
        .L = glm::vec3(0.0, 0.0, 0.0)   // Angular Momentum
    };
    
    glEnable(GL_DEPTH_TEST);
    double previous_time = glfwGetTime();
    double elapsed_time = 0.0;
    float angle = 0.0f;

    // Camera stuff
    double previous_xpos, previous_ypos;
    glfwGetCursorPos(window, &previous_xpos, &previous_ypos);
    double theta, phi;


    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();


    //impulses.push({0, 0.2, {0.0, 0.0, 0.0}, {100.0, 100.0, 100.0}});

    while(!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        double current_time = glfwGetTime();
        double delta = current_time - previous_time;
        previous_time = current_time;
        

        elapsed_time += delta;
        if (elapsed_time > 0.01)
        {
            // Run simulation

            ode(rb, elapsed_time);
            //ode(rb2, elapsed_time);

            // Marches the simulation forward by elapsed_time
            world.update(elapsed_time);

            glm::mat4 model = world.get_world_matrix(id);

            draw_ui(rb);

            double current_xpos, current_ypos;
            glfwGetCursorPos(window, &current_xpos, &current_ypos);

            double xdelta = current_xpos - previous_xpos;
            double ydelta = current_ypos - previous_ypos;
            previous_xpos = current_xpos;
            previous_ypos = current_ypos;
            
            if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
            {
                theta += xdelta;
                phi += ydelta; 
            }
            
            
            if (phi >= 89.0) phi = 89.0;
            if (phi <= -89.0) phi = -89.0;


            double theta_radian = glm::radians(theta);
            double phi_radian = glm::radians(phi);

            glm::mat4 view = glm::lookAt(glm::vec3(cam_radius * sin(-theta_radian) * cos(phi_radian), cam_radius * sin(phi_radian), cam_radius * cos(-theta_radian) * cos(phi_radian)), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 1.0, 0.0));

            // Display objects
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glUseProgram(program);
            glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view));

            sphere->draw(program, model);
            plane->draw(program, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -3.0f, 0.0)));
            right_plane->draw(program, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(5.0f, 2.0f, 0.0f)), glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
            
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            glfwSwapBuffers(window);
 
            elapsed_time = 0.0;
        }
    }

    glBindBuffer(0, GL_ARRAY_BUFFER);
    glBindBuffer(0, GL_ELEMENT_ARRAY_BUFFER);
    glBindVertexArray(0);

    glUseProgram(0);
    glDeleteProgram(program);
    
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}


void draw_ui(RigidBody& rb)
{
    // Build UI
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoCollapse;

    //const ImGuiViewport* main_viewport = ImGui::GetMainViewport();
    //ImGui::SetNextWindowPos(ImVec2(main_viewport->WorkPos.x + 100, main_viewport->WorkPos.y + 20), ImGuiCond_FirstUseEver);
    //ImGui::SetNextWindowSize(ImVec2(550, 680), ImGuiCond_FirstUseEver);

    if(!ImGui::Begin("Stats", nullptr, window_flags))
    {
        ImGui::End();
        return;
    }


    ImGui::Text("Mass: %f", rb.mass);
    ImGui::Spacing();
    ImGui::Text("Position: %f %f %f", rb.x.x, rb.x.y, rb.x.z);
    ImGui::Spacing();
    ImGui::Text("Orientation (Quaternion): %f %f %f %f", rb.q.w, rb.q.x, rb.q.y, rb.q.z);
    ImGui::Spacing();
    ImGui::Text("Orientation (Axis Angle): TODO");
    ImGui::Spacing();
    ImGui::Text("P: %f %f %f |P|: %f", rb.P.x, rb.P.y, rb.P.z, glm::length(rb.P));
    ImGui::Spacing();
    ImGui::Text("L: %f %f %f |L|: %f", rb.L.x, rb.L.y, rb.L.z, glm::length(rb.L));
    ImGui::Spacing();
    ImGui::Text("V: %f %f %f |V|: %f", rb.v.x, rb.v.y, rb.v.z, glm::length(rb.v));
    ImGui::Spacing();
    ImGui::Text("Omega: %f %f %f |Omega|: %f", rb.omega.x, rb.omega.y, rb.omega.z, glm::length(rb.omega));

    ImGui::End();
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