#include <iostream>
#include <render/render.h>
#include <physics/physics.h>
#include <memory>

double cam_radius = 5.0;
const int WIN_WIDTH = 1920;
const int WIN_HEIGHT = 1080;

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    cam_radius -= 0.2 * yoffset;
}

int main()
{

    GLFWwindow* window = init_window(WIN_WIDTH, WIN_HEIGHT, "Splines");
    glfwSetScrollCallback(window, scroll_callback);

    glClearColor(0.3, 0.3, 0.3, 1.0);
    glEnable(GL_DEPTH_TEST);

    unsigned int shader;
    if (!load_shader("../shader/line.vert", "../shader/line.frag", &shader))
    {
        std::cerr << "Failed to load shader" << std::endl;
        exit(EXIT_FAILURE);
    }

    glm::mat4 projection = glm::perspective(glm::radians(90.0f), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 10.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    glLineWidth(5.0);
    std::vector<glm::vec3> points = 
    {
        {0.0, 0.0, -1.0},
        {10.0, 10.0, -1.0}
    };

    std::shared_ptr<Geometry> curve = GeometryFactory::load_curve(points);
    
    // Delta time stuff
    double previous_time = glfwGetTime();
    double elapsed_time = 0.0;

    // Camera stuff
    double previous_xpos, previous_ypos;
    glfwGetCursorPos(window, &previous_xpos, &previous_ypos);
    double theta = 0.0, phi = 0.0;
    while(!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        double current_time = glfwGetTime();
        double delta = current_time - previous_time;
        previous_time = current_time;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shader);
        curve->draw(shader, glm::mat4(1.0));

        glfwSwapBuffers(window);
    }

    deinit_window(window);

    return 0;
}