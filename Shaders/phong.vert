#version 460 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out vec3 fragPos;
out vec3 normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

void main()
{
    vec4 position = vec4(aPos, 1.0);
    vec4 worldPos = model * position;
    fragPos = worldPos.xyz / worldPos.w;
    gl_Position = proj * view * worldPos;
    normal = mat3(transpose(inverse(model))) * aNormal;
}