#ifndef MUJOCO_CLIENT_H
#define MUJOCO_CLIENT_H

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <memory>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>
#include <boost/format.hpp>

namespace mujoco_client {

struct SensorInfo {
    int adr;
    int dim;
};

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
    bool isCollision(const std::vector<double> &config);

    /** @brief Check if the robot is in collision in the current configuration.
     *  @return True if the robot is in collision, false otherwise.
     */
    bool isCollision() const;

    /** @brief Initialize GLFW for rendering. */
    void initializeGlfw();

    /** @brief Render the robot in the current configuration. */
    void render();

    /** @brief Set the force and torque applied to the robot.
     *  @param ft The force and torque applied to the robot [Fx, Fy, Fz, Mx, My, Mz].
     */
    // void setFT(const std::vector<double> &ft) const;

    /** @brief Get the resultant force of the base. */
    // std::vector<double> getBaseFT() const;

    /** @brief Get the rotation matrix from the body-fixed frame to the inertial frame. */
    Eigen::Vector3d getBaseAngVel() const;
    Eigen::Vector3d getBaseAcc() const;
    Eigen::Vector3d getBasePos() const;
    Eigen::Quaternion<double> getBaseQuat() const;
    Eigen::Matrix3d getBaseRotation() const;

    // GLFW callback function
    void mouse_button_callback_impl(GLFWwindow* window, int button, int action, int mods);
    void cursor_position_callback_impl(GLFWwindow* window, double xpos, double ypos);
    void scroll_callback_impl(GLFWwindow* window, double xoffset, double yoffset);
    [[nodiscard]] GLFWwindow* getWindow() const { return window_.get(); }

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

    /** @brief Map the force and torque from the base to thrust of the rotors.
     *  @param ft The force and torque applied to the base [Fx, Fy, Fz, Mx, My, Mz].
     *  @return The thrust of the rotors [T1, T2, T3, T4, T5, T6].
     */
    // std::vector<double> inverseMapFT(const std::vector<double> &ft) const;

    /** @brief get the sensor information of the robot.
     *  @param sensor_name The name of the sensor.
     *  @return The sensor information [data_address, data_size].
     */
    SensorInfo getSensorInfo(const char* sensor_name) const;

    /** @brief Print the collision information of the robot.
     *  The information includes the number of contacts, and the contact geometry.
     */
    void printCollisionInfo() const;

    /** @brief Print the sensor information of the robot. */
    void printSensorInfo() const;

    // void drawArrow(const Eigen::Vector3d & from,
    //                           const Eigen::Vector3d & to,
    //                           double shaft_diam = 0.02);

    // void showBaseAcc();

protected:

    const char* model_path_;

    const double sim_time_step_{0.01};

    // MuJoCo structures for rendering and simulation
    mjvScene scene_;
    mjrContext context_;
    mjvCamera cam_;
    mjvOption opt_;


    // Model parameters
    double yawOffset_{0.0};
    double jointOffset_{-M_PI/6.0};
    // Eigen::Matrix<double, 6, 6> forwardForceMap_;
    // Eigen::Matrix<double, 6, 6> inverseForceMap_;

    std::vector<double> camPoint_{0.0,-1.0,2.0};
    double camDis_{4.0};
    double camAzimuth_{30.0};
    double camElevation_{0.0};

    const char* baseBodyName_{"base_link"};
    const char* baseJointName_{"free_joint"};
    const char* jointName_{"operator_1_joint"};
    std::vector<const char*> motorNames_{"thrust0", "thrust1", "thrust2",
                                        "thrust3", "thrust4", "thrust5"};
    int rootQPos_{-1};
    int jointQPos_{-1};
    std::vector<int> motorIDs_;
    int baseId_{-1};

    // Sensor data
    SensorInfo sGyro_;
    SensorInfo sAcc_;
    SensorInfo sQuat_;
    SensorInfo sPos_;

    // visulaization elements
    std::vector<mjvGeom> geoms_;

    // MuJoCo structures for simulation
    std::shared_ptr<mjModel> model_;
    std::shared_ptr<mjData> data_;

    // GLFW window and context
    std::shared_ptr<GLFWwindow> window_;
    double lastMouseX_{0.0};
    double lastMouseY_{0.0};
    bool buttonLeft_{false};
    bool buttonMiddle_{false};
    bool buttonRight_{false};
};
} // namespace mujoco_client

#endif //MUJOCO_CLIENT_H
