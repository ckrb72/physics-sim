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
#include "util.h"

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


void draw_ui(const BodyInfo& rb);

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

    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    std::shared_ptr<Geometry> sphere = GeometryFactory::load_sphere(1.0, 3);
    std::shared_ptr<Geometry> plane = GeometryFactory::load_plane(10.0, 10.0);
    std::shared_ptr<Geometry> right_plane = GeometryFactory::load_plane(10.0, 10.0);

    PhysicsWorld world;
    int32_t sphere_body = world.create_body(std::make_shared<BoxShape>(glm::vec3(1.0f)), 100.0, PhysicsLayer::DYNAMIC);
    //int32_t bottom_plane_body = world.create_body(std::make_shared<PlaneShape>(glm::vec3(10.0, 10.0, 10.0)), glm::vec3(0.0, -3.0, 0.0), glm::angleAxis(0.0f, glm::vec3(1.0, 0.0, 0.0)), 1.0, PhysicsLayer::STATIC);
    //int32_t testing_stuff = world.create_body(std::make_shared<PlaneShape>(glm::vec3(10.0, 10.0, 10.0)), glm::vec3(5.0, 2.0, 0.0), glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0, 0.0, 1.0)), 1.0, PhysicsLayer::STATIC);

    
    std::cout << "Rigid Bodies Created..." << std::endl;

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

    glEnable(GL_DEPTH_TEST);
    double previous_time = glfwGetTime();
    double elapsed_time = 0.0;
    float angle = 0.0f;

    // Camera stuff
    double previous_xpos, previous_ypos;
    glfwGetCursorPos(window, &previous_xpos, &previous_ypos);
    double theta = 0.0, phi = 0.0;


    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    world.set_linear_velocity(sphere_body, glm::vec3(0.0f, 9.8f, 0.0f));
    world.set_global_force(glm::vec3(0.0, -9.8, 0.0));


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

            // Marches the simulation forward by elapsed_time
            world.update(elapsed_time);

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

            sphere->draw(program, world.get_world_matrix(sphere_body));
            //plane->draw(program, world.get_world_matrix(bottom_plane_body));
            //right_plane->draw(program, world.get_world_matrix(testing_stuff));

            draw_ui(world.get_info(sphere_body));
            ImGui::EndFrame();

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


void draw_ui(const BodyInfo& rb)
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
    ImGui::Text("Position: %f %f %f", rb.position.x, rb.position.y, rb.position.z);
    ImGui::Spacing();
    ImGui::Text("Orientation (Quaternion): %f %f %f %f", rb.orientation.w, rb.orientation.x, rb.orientation.y, rb.orientation.z);
    ImGui::Spacing();
    ImGui::Text("Linear Momentum: %f %f %f |LM|: %f", rb.linear_momentum.x, rb.linear_momentum.y, rb.linear_momentum.z, glm::length(rb.linear_momentum));
    ImGui::Spacing();
    ImGui::Text("Angular Momentum: %f %f %f |AM|: %f", rb.angular_momentum.x, rb.angular_momentum.y, rb.angular_momentum.z, glm::length(rb.angular_momentum));
    ImGui::Spacing();
    ImGui::Text("Force: %f %f %f |F|: %f", rb.force.x, rb.force.y, rb.force.z, glm::length(rb.force));
    ImGui::Spacing();
    ImGui::Text("Impulse: %f %f %f |Impulse|: %f", rb.impulse.x, rb.impulse.y, rb.impulse.z, glm::length(rb.impulse));
    ImGui::Spacing();
    ImGui::Text("Torque: %f %f %f |F|: %f", rb.torque.x, rb.torque.y, rb.torque.z, glm::length(rb.torque));
    ImGui::Spacing();
    ImGui::Text("Force: %f %f %f |F|: %f", rb.torque_impulse.x, rb.torque_impulse.y, rb.torque_impulse.z, glm::length(rb.torque_impulse));
    ImGui::Spacing();

    ImGui::End();
}
