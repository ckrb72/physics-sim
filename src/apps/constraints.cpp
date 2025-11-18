#include <iostream>
#include <render/render.h>
#include <render/engine.h>
#include <physics/physics.h>

const int WIN_WIDTH = 1920;
const int WIN_HEIGHT = 1080;

double cam_radius = 5.0;
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    cam_radius -= 0.2 * yoffset;
}

int main()
{
    GLFWwindow* window = init_window(WIN_WIDTH, WIN_HEIGHT, "Constraints");

    glClearColor(0.3, 0.3, 0.3, 1.0);

    PhysicsWorld world;
    int32_t box_body = world.create_body(std::make_shared<OBBShape>(glm::vec3(0.125, 2.0, 0.125)), 10.0, PhysicsLayer::DYNAMIC);
    int32_t weight_body = world.create_body(std::make_shared<OBBShape>(glm::vec3(0.5)), glm::vec3(0.0, -2.5, 0.0), glm::angleAxis(0.0f, glm::vec3(1.0, 0.0, 0.0)), 100.0, PhysicsLayer::DYNAMIC);

    std::shared_ptr<Geometry> box_mesh = GeometryFactory::load_rect(0.25f, 4.0f, 0.25f);
    std::shared_ptr<Geometry> weight_mesh = GeometryFactory::load_rect(1.0f, 1.0f, 1.0f);
    
    glfwSetScrollCallback(window, scroll_callback);

    unsigned int shader;
    if (!load_shader("../shader/default.vert", "../shader/default.frag", &shader))
    {
        std::cerr << "Failed to load shader" << std::endl;
        exit(EXIT_FAILURE);
    }

    glm::mat4 projection = glm::perspective(glm::radians(90.0f), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 50.0f);
    glUseProgram(shader);
    glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, glm::value_ptr(projection));


    EngineTime time;
    time.init();

    // Camera stuff
    double previous_xpos, previous_ypos;
    glfwGetCursorPos(window, &previous_xpos, &previous_ypos);
    double theta = 0.0, phi = 0.0;
    while(!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        time.update();

        // Camera
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


        //world.update(delta);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shader);
        glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, glm::value_ptr(view));

        box_mesh->draw(shader, world.get_world_matrix(box_body));
        weight_mesh->draw(shader, world.get_world_matrix(weight_body));

        glfwSwapBuffers(window);
    }

    deinit_window(window);
}