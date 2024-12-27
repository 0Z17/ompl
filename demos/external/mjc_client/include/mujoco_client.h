#ifndef MUJOCO_CLIENT_H
#define MUJOCO_CLIENT_H

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <memory>
#include <Eigen/Geometry>
#include <iostream>


namespace mujoco_client {

class MujocoClient {
public:
    explicit MujocoClient(const char* model_path);
    ~MujocoClient() = default;

    /** @brief Set the comfig of the robot.
     *  @param config The configuration of the robot [x,y,z,psi,theta].
     *  The config consists of the pos of the base [x,y,z], the yaw of the base [psi],
     *  and the joint angles of the [theta]
     *
     */
    void setConfig(const std::vector<double> &config);

    /** @brief Check if the robot is in collision.
     *  @param config The configuration of the robot [x,y,z,psi,theta].
     *  @return True if the robot is in collision, false otherwise.
     */
    bool checkCollision(const std::vector<double> &config);

    /** @brief Check if the robot is in collision in the current configuration.
     *  @return True if the robot is in collision, false otherwise.
     */
    bool checkCollision() const;

    /** @brief Initialize GLFW for rendering. */
    void initializeGlfw();

    /** @brief Render the robot in the current configuration. */
    void render();

    // GLFW callback function
    void mouse_button_callback_impl(GLFWwindow* window, int button, int action, int mods);
    void cursor_position_callback_impl(GLFWwindow* window, double xpos, double ypos);
    void scroll_callback_impl(GLFWwindow* window, double xoffset, double yoffset);

    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
    {
        if (auto *client = static_cast<MujocoClient *>(glfwGetWindowUserPointer(window))) {
            client->mouse_button_callback_impl(window, button, action, mods);
            }
    };
    static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
    {
        if (auto *client = static_cast<MujocoClient *>(glfwGetWindowUserPointer(window))) {
            client->cursor_position_callback_impl(window, xpos, ypos);
            }
    };
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
        if (auto *client = static_cast<MujocoClient *>(glfwGetWindowUserPointer(window))) {
            client->scroll_callback_impl(window, xoffset, yoffset);
        }
    };


    // Utility functions
    static Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
        return Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    }

protected:

    const char* model_path_;

    const double sim_time_step_{0.01};

    // MuJoCo structures for rendering and simulation
    mjvScene scene_;
    mjrContext context_;
    mjvCamera cam_;
    mjvOption opt_;

    std::vector<double> camPoint_{0.0,0.0,1.0};
    double camDis_{2.0};

    const char* baseName_{"robot"};
    const char* jointName_{"operator_1_joint"};
    int rootQPos_{-1};
    int jointQPos_{-1};

    // MuJoCo structures for simulation
    std::shared_ptr<mjModel> model_;
    std::shared_ptr<mjData> data_;

    // GLFW window and context
    std::shared_ptr<GLFWwindow> window_;
    bool isMousePressed_{false};
    double lastMouseX_{0.0};
    double lastMouseY_{0.0};
};
} // namespace mujoco_client

#endif //MUJOCO_CLIENT_H
