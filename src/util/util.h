#pragma once
#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

#define NOT_IMPLEMENTED() do { std::cout << __FILE__ << ":" << __LINE__ << "  << NOT IMPLEMENTED >>" << std::endl;} while(0);

void print_mat3(const glm::mat3& m);
void print_vec3(const glm::vec3& v);
void print_quat(const glm::quat& q);
//void print_rigid_body(const RigidBody& rb);