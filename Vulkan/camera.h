#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLFW_INCLUDE_VULKAN
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

namespace cam {

    struct CameraInfo {
        glm::vec3 position_;
        glm::vec3 center_;
        glm::vec3 up_vector_;

        float view_angle_;
        float aspect_;
        float near_;
        float far_;

        float speed_;
        float rotate_speed_;
    };

    class Camera {
    public:
        explicit Camera(const CameraInfo &info) :
                position_(info.position_),
                direction_(glm::normalize(info.center_ - info.position_)),
                up_vector_(info.up_vector_),
                rotate_speed_(info.rotate_speed_),
                proj_matrix(glm::perspective(glm::radians(info.view_angle_), info.aspect_, info.near_, info.far_)) {

            speed_ = direction_.length() * info.speed_;
        };

        [[nodiscard]] glm::mat4 getViewMatrix() const { return glm::lookAt(position_, position_ + direction_, up_vector_);};
        [[nodiscard]] glm::mat4 getProjMatrix() const { return proj_matrix;};

        void update(float time, GLFWwindow *window) {

            int window_width = 0, window_height = 0;
            glfwGetWindowSize(window, &window_width, &window_height);

            //Rotating
            if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
                const float dx = time * rotate_speed_;
                RotateHorizontal(glm::radians((-dx) / static_cast<float>(window_width)));
            }
            if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
                const float dx = time * rotate_speed_;
                RotateHorizontal(glm::radians((dx) / static_cast<float>(window_width)));
            }
            if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
                const float dy = time * rotate_speed_;
                RotateVertical(glm::radians(dy / static_cast<float>(window_height)));
            }
            if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
                const float dy = time * rotate_speed_;
                RotateVertical(glm::radians((-dy) / static_cast<float>(window_height)));
            }

            // Moving
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
                position_ += (speed_ * time) * direction_;
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
                position_ -= (speed_ * time) * direction_;
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
                position_ -= glm::cross(direction_, up_vector_) * (speed_ * time);
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
                position_ += glm::cross(direction_, up_vector_) * (speed_ * time);
        }

    private:

        void RotateHorizontal(float angle) {
            direction_ = glm::rotate(glm::mat4(1.f), angle, up_vector_) * glm::vec4(direction_, 0.f);
        }

        void RotateVertical(float angle) {
            auto axis = glm::cross(direction_, up_vector_);
            direction_ = glm::rotate(glm::mat4(1.f), angle, axis) * glm::vec4(direction_, 0.f);
        }

        glm::vec3 position_;
        glm::vec3 direction_;
        glm::vec3 up_vector_;

        const glm::mat4 proj_matrix;
        const float rotate_speed_;
        float speed_;
    };

}
