#version 410 core

in vec3 f_norm;
in vec3 f_pos;

out vec4 final_color;

uniform float colliding;

void main()
{
    final_color = vec4(colliding, 0.0, 1.0, 1.0);
}