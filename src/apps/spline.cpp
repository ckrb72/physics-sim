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

std::vector<glm::vec3> generate_spline(const std::vector<glm::vec3>& points, uint32_t subdivisions);

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

    glfwSwapInterval(0);

    glm::mat4 projection = glm::perspective(glm::radians(90.0f), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 50.0f);
    glUseProgram(shader);
    glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    glLineWidth(20.0);
    std::vector<glm::vec3> points = 
    {
        {-3.0f, 0.0f, 0.0f},
        {0.0f, 3.0f, 0.0f},
        {3.0f, 0.0f, 0.0f},
        {0.0f, -7.0f, 1.0f},
        {0.0f, 0.0f, -3.0f}
    };

    std::vector<glm::vec3> interpolated_points = generate_spline(points, 20);
    std::shared_ptr<Geometry> curve = GeometryFactory::load_curve(interpolated_points);
    
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


        // Render
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUseProgram(shader);
        glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, glm::value_ptr(view));

        curve->draw(shader, glm::mat4(1.0));
        
        glfwSwapBuffers(window);
    }

    deinit_window(window);

    return 0;
}

std::vector<glm::vec3> generate_spline(const std::vector<glm::vec3>& points, uint32_t subdivisions)
{
    std::vector<glm::vec3> spline_points;

    // Can't have 0 subdivisions
    if (subdivisions == 0) subdivisions = 1;


    float tension = 1.0f;

    // TODO: Fix this...
    glm::mat4 spline_mat = 
    {
        0.0f, 1.0f, 0.0f, 0.0f,
        -tension, 0.0f, tension, 0.0f,
        2.0f * tension, tension - 3.0f, 3.0f - (2.0f * tension), -tension,
        -tension, 2.0f - tension, tension - 2.0f, tension
    };
    
    spline_mat = glm::transpose(spline_mat);

    for (int i = 0; i < points.size() - 1; i++)
    {   
        glm::vec3 prev = i == 0 ? points[i] : points[i - 1];
        glm::vec3 start = points[i];
        glm::vec3 end = points[i + 1];
        glm::vec3 next = i == points.size() - 2 ? points[i + 1] : points[i + 2];

        // TODO: Fix this...
        glm::mat4x3 point_mat = 
        {
            prev,
            start,
            end,
            next
        };

        for (int sub = 0; sub <= subdivisions; sub++)
        {
            float t = (float)sub / (float)subdivisions;
            float t_sqr = t * t;
            float t_cube = t * t * t;
            
            glm::vec3 point = (-0.5f * tension * t_cube + tension * t_sqr - 0.5f * tension * t) * prev +
                            (1.0f + 0.5f * t_sqr * (tension - 6.0f) + 0.5f * t_cube * (4.0f - tension)) * start +
                            (0.5f * t_cube * (tension - 4.0f) + 0.5f * tension * t - (tension - 3.0f) * t_sqr) * end +
                            (-0.5f * tension * t_sqr + 0.5f * tension * t_cube) * next;
            spline_points.push_back(point);
        }   
    }

    return spline_points;
}