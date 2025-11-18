#include <iostream>
#include <physics/physics.h>
#include <render/render.h>

double cam_radius = 5.0;
const int WIN_WIDTH = 1920;
const int WIN_HEIGHT = 1080;

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    cam_radius -= 0.2 * yoffset;
}

int main()
{
    GLFWwindow* window = init_window(WIN_WIDTH, WIN_HEIGHT, "OBB");
    glfwSetScrollCallback(window, scroll_callback);

    glClearColor(0.3, 0.3, 0.3, 1.0);
    glEnable(GL_DEPTH_TEST);

    std::shared_ptr<Geometry> box_a = GeometryFactory::load_rect(2.0f, 2.0f, 2.0f);
    std::shared_ptr<Geometry> box_b = GeometryFactory::load_rect(2.0f, 2.0f, 2.0f);
    PhysicsWorld world;
    int32_t box_a_body = world.create_body(std::make_shared<OBBShape>(glm::vec3(1.0f, 1.0f, 1.0f)), glm::vec3(-5.0f, 0.0f, 0.0f), glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f)), 100.0f, PhysicsLayer::DYNAMIC);
    world.set_linear_velocity(box_a_body, glm::vec3(1.0f, 0.0f, 0.0f));
    int32_t box_b_body = world.create_body(std::make_shared<OBBShape>(glm::vec3(1.0f, 1.0f, 1.0f)), glm::vec3(5.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 100.0f, PhysicsLayer::DYNAMIC);
    world.set_linear_velocity(box_b_body, glm::vec3(-1.0f, 0.0f, 0.0f));

    unsigned int program;
    if(!load_shader("../shader/collision_view.vert", "../shader/collision_view.frag", &program))
    {
        std::cout << "Failed to load shader" << std::endl;

        exit(EXIT_FAILURE);
    }

    glm::mat4 perspective = glm::perspective(glm::radians(90.0f), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 50.0f);
    glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0));
    
    glUseProgram(program);
    glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, glm::value_ptr(perspective));
    glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view));

    // Delta time stuff
    double previous_time = glfwGetTime();
    double elapsed_time = 0.0;

    // Camera stuff
    double previous_xpos, previous_ypos;
    glfwGetCursorPos(window, &previous_xpos, &previous_ypos);
    double theta = 0.0, phi = 0.0;
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        double current_time = glfwGetTime();
        double delta = current_time - previous_time;
        previous_time = current_time;

        world.update(delta);

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
        glUseProgram(program);
        glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view));

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        glUniform1f(glGetUniformLocation(program, "colliding"), (world.is_colliding(box_a_body, box_b_body)) ? 1.0f : 0.0f);
        box_a->draw(program, world.get_world_matrix(box_a_body));
        box_b->draw(program, world.get_world_matrix(box_b_body));
        
        glfwSwapBuffers(window);
    }
}