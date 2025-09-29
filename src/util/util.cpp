#include "util.h"

void print_mat3(const glm::mat3& m)
{
    for (int r = 0; r < 3; r++)
    {
        std::cout << "| ";
        for (int c = 0; c < 3; c++)
        {
            std::cout << m[r][c] << " ";
        }
        std::cout << "|" << std::endl;
    }
}

void print_vec3(const glm::vec3& v)
{
    std::cout << "[ " << v.x << " " << v.y << " " << v.z << " ]" << std::endl;
}

void print_quat(const glm::quat& q)
{
    double theta = acos(q.w) * 2.0;
    double s = sin(theta / 2.0);
    std::cout << "Theta: " << theta * 180.0 / 3.1415 << " [ " << q.x / s << " " << q.y / s << " " << q.z / s << " ]" << std::endl;
    std::cout << "| " << q.w << " " << q.x << " " << q.y << " " << q.z << " |" << std::endl;
}

/*void print_rigid_body(const RigidBody& rb)
{
    std::cout << std::endl;
    std::cout << "Mass: " << rb.mass << std::endl;
    std::cout << "Ibody: " << std::endl;
    print_mat3(rb.Ibody);

    std::cout << "Pos: " << std::endl;
    print_vec3(rb.x);

    std::cout << "Orientation: " << std::endl;
    print_quat(rb.q);

    std::cout << "Linear Momentum: " << std::endl;
    print_vec3(rb.P);

    std::cout << "Angular Momentum: " << std::endl;
    print_vec3(rb.L);

    std::cout << "Linear Velocity: " << std::endl;
    print_vec3(rb.v);

    std::cout << "Angular Velocity: " << std::endl;
    print_vec3(rb.omega);

    std::cout << std::endl;
}*/