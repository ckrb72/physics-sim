#version 410 core

in vec3 f_norm;
in vec3 f_pos;

uniform vec3 light_pos;
uniform vec3 object_color;

vec3 light_color = vec3(0.6, 0.6, 0.6);
float ambient = 0.2;

out vec4 final_color;

void main()
{
    vec3 light_dir = normalize(light_pos - f_pos);
    vec3 norm = normalize(f_norm);
    float diffuse = max(dot(light_dir, norm), 0.0);
    vec3 diffuse_strength = diffuse * light_color;
    vec3 ambient_strength = ambient * light_color;

    // final_color = vec4((diffuse_strength + ambient_strength) * object_color, 1.0);
    final_color = vec4(f_norm, 1.0);
}