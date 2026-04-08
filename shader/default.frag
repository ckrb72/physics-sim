#version 410 core

in vec3 f_norm;
in vec3 f_pos;

uniform vec3 light_pos;
uniform vec3 object_color;

vec3 light_color = vec3(1.0, 1.0, 1.0);
float ambient = 0.2;

out vec4 final_color;

void main()
{
    vec3 light_dir = normalize(light_pos - f_pos);
    vec3 norm = normalize(f_norm);
    float diffuse = max(dot(light_dir, norm), 0.0);
    // vec3 diffuse_strength = diffuse * light_color;
    // vec3 ambient_strength = ambient * light_color;

    float d = length(light_pos - f_pos);
    float attenuation = 1.0 / (1.0 + (0.027 * d) + (0.0028 * d * d));

    vec3 lighting = (ambient + (diffuse * attenuation)) * light_color;

    final_color = vec4(lighting * object_color, 1.0);
    // final_color = vec4(f_norm, 1.0);
}