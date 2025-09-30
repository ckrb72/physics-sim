#version 410 core
layout(location = 0) in vec3 v_pos;

uniform mat4 projection;
uniform mat4 view;

void main()
{
    gl_Position = projection * vec4(v_pos, 1.0);
}