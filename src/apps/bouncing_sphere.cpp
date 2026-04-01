#include <iostream>
#include <fstream>
#include <string>
#include <queue>

#include <render/render.h>
#include <render/engine.h>
#include <physics/physics.h>

double cam_radius = 5.0;

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    cam_radius -= 0.2 * yoffset;
}


void glfw_error_fun(int error, const char* err_desc)
{
    std::cout << err_desc << std::endl;
}

const int WIN_WIDTH = 1920;
const int WIN_HEIGHT = 1080;


void draw_ui(const BodyInfo& rb);

int main()
{
    GLFWwindow* window = init_window(WIN_WIDTH, WIN_HEIGHT, "Bouncing Ball");
    glfwSetScrollCallback(window, scroll_callback);

    std::shared_ptr<Geometry> sphere = GeometryFactory::load_sphere(1.0, 3);
    std::shared_ptr<Geometry> plane = GeometryFactory::load_plane(10.0, 10.0);
    std::shared_ptr<Geometry> cube = GeometryFactory::load_rect(1.0, 1.0, 1.0);

    PhysicsWorld world;
    BodyId sphere_body = world.create_body(PhysicsShape::MakeSphere(1.0), PhysicsMaterial{ .restitution = 0.8f }, Vector3(1.0, 0.0, 0.0), 100.0, PhysicsLayer::DYNAMIC);
    BodyId cube_body = world.create_body(PhysicsShape::MakeOBB(Vector3(0.5, 0.5, 0.5)), Vector3(-1.0, 0.0, 0.0), 1.0, PhysicsLayer::DYNAMIC);
    BodyId bottom_plane_body = world.create_body(PhysicsShape::MakePlane(Eigen::Vector3d(10.0, 10.0, 10.0)), Eigen::Vector3d(0.0, -3.0, 0.0), Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d(1.0, 0.0, 0.0))), 1.0, PhysicsLayer::STATIC);
    BodyId testing_stuff = world.create_body(PhysicsShape::MakePlane(Eigen::Vector3d(10.0, 10.0, 10.0)), Eigen::Vector3d(5.0, 2.0, 0.0), Eigen::Quaterniond(Eigen::AngleAxisd(DegreesToRadians(90.0), Eigen::Vector3d(0.0, 0.0, 1.0))), 1.0, PhysicsLayer::STATIC);
    BodyId left_plane = world.create_body(PhysicsShape::MakePlane(Eigen::Vector3d(10.0, 10.0, 10.0)), Eigen::Vector3d(-5.0, 2.0, 0.0), Eigen::Quaterniond(Eigen::AngleAxisd(DegreesToRadians(-90.0), Eigen::Vector3d(0.0, 0.0, 1.0))), 1.0, PhysicsLayer::STATIC);

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
    glUniform3fv(glGetUniformLocation(program, "object_color"), 1, glm::value_ptr(glm::vec3(1.0, 1.0, 1.0)));

    EngineTime time;
    time.init();
    double elapsed_time = 0.0;

    // Camera stuff
    double previous_xpos, previous_ypos;
    glfwGetCursorPos(window, &previous_xpos, &previous_ypos);
    double theta = 0.0, phi = 0.0;

    world.set_linear_velocity(sphere_body, Vector3(-1.0, 9.8, 0.0));
    // world.set_angular_velocity(sphere_body, Vector3(0.0, 2.0, 0.0));
    world.set_gravity({ 0.0, 0.0, 0.0, 0.0, -9.8, 0.0});


    while(!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        time.update();
        

        elapsed_time += time.delta();
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

            glm::vec3 cam_pos = glm::vec3(cam_radius * sin(-theta_radian) * cos(phi_radian), cam_radius * sin(phi_radian), cam_radius * cos(-theta_radian) * cos(phi_radian));
            glm::mat4 view = glm::lookAt(cam_pos, glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 1.0, 0.0));

            // Display objects
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glUseProgram(program);
            glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniform3fv(glGetUniformLocation(program, "light_pos"), 1, glm::value_ptr(cam_pos));

            // std::cout << world.get_world_matrix(sphere_body) << std::endl;
            sphere->draw(program, EigenMatrixToFloatArray(world.get_world_matrix(sphere_body)));
            plane->draw(program, EigenMatrixToFloatArray(world.get_world_matrix(bottom_plane_body)));
            plane->draw(program, EigenMatrixToFloatArray(world.get_world_matrix(testing_stuff)));
            plane->draw(program, EigenMatrixToFloatArray(world.get_world_matrix(left_plane)));
            cube->draw(program, EigenMatrixToFloatArray(world.get_world_matrix(cube_body)));

            // draw_ui(world.get_info(sphere_body));

            glfwSwapBuffers(window);
 
            elapsed_time = 0.0;
        }
    }

    glBindBuffer(0, GL_ARRAY_BUFFER);
    glBindBuffer(0, GL_ELEMENT_ARRAY_BUFFER);
    glBindVertexArray(0);

    glUseProgram(0);
    glDeleteProgram(program);

    deinit_window(window);

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
    ImGui::Text("Position: %f %f %f", rb.position.x(), rb.position.y(), rb.position.z());
    ImGui::Spacing();
    ImGui::Text("Orientation (Quaternion): %f %f %f %f", rb.orientation.w(), rb.orientation.x(), rb.orientation.y(), rb.orientation.z());
    ImGui::Spacing();
    ImGui::Text("Linear Momentum: %f %f %f |LM|: %f", rb.linear_momentum.x(), rb.linear_momentum.y(), rb.linear_momentum.z(), rb.linear_momentum.norm());
    ImGui::Spacing();
    ImGui::Text("Angular Momentum: %f %f %f |AM|: %f", rb.angular_momentum.x(), rb.angular_momentum.y(), rb.angular_momentum.z(), rb.angular_momentum.norm());
    ImGui::Spacing();
    ImGui::Text("Force: %f %f %f |F|: %f", rb.force.x(), rb.force.y(), rb.force.z(), rb.force.norm());
    ImGui::Spacing();
    ImGui::Text("Impulse: %f %f %f |Impulse|: %f", rb.impulse.x(), rb.impulse.y(), rb.impulse.z(), rb.impulse.norm());
    ImGui::Spacing();
    ImGui::Text("Torque: %f %f %f |F|: %f", rb.torque.x(), rb.torque.y(), rb.torque.z(), rb.torque.norm());
    ImGui::Spacing();
    ImGui::Text("Force: %f %f %f |F|: %f", rb.torque_impulse.x(), rb.torque_impulse.y(), rb.torque_impulse.z(), rb.torque_impulse.norm());
    ImGui::Spacing();

    ImGui::End();

    ImGui::EndFrame();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
