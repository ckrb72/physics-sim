#version 410 core

in vec3 f_norm;
in vec3 f_pos;

out vec4 final_color;

void main()
{
    final_color = vec4(f_norm, 1.0);
}