#version 410 core

in vec3 f_norm;
in vec3 f_pos;

out vec4 final_color;

void main()
{
    vec3 norm = (f_norm + 1.0) / 2.0;
    final_color = vec4(norm, 1.0);
}