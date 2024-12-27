#include <iostream>
#include "mujoco_client.h"

namespace mc = mujoco_client;

mc::MujocoClient::MujocoClient(const char* model_path)
{
    // Initialize the GLFW
    initializeGlfw();

    // Load the model
    char error[1000] = "";
    mjModel* raw_model = mj_loadXML(model_path, nullptr, error, 1000);
    if (!raw_model) {
        throw std::runtime_error("Could not load model");
    }
    model_path_ = model_path;

    model_ = std::shared_ptr<mjModel>(raw_model, [](mjModel* model) {
            if (model) {
                mj_deleteModel(model);
            }
        });

    // Create a MuJoCo simulation
    mjData* raw_data = mj_makeData(model_.get());
    if (!raw_data) {
        throw std::runtime_error("Could not create simulation data");
    }

    data_ = std::shared_ptr<mjData>(raw_data, [](mjData* data) {
            if (data) {
                mj_deleteData(data);
            }
        });

    // Initialize MuJoCo rendering structures
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);

    scene_.maxgeom = model_->ngeom;
    mjv_makeScene(model_.get(), &scene_, scene_.maxgeom);
    mjr_makeContext(model_.get(), &context_, mjFONTSCALE_150);

    // Set default camera position
    cam_.lookat[0] = camPoint_[0];
    cam_.lookat[1] = camPoint_[1];
    cam_.lookat[2] = camPoint_[2];
    cam_.distance = camDis_;

    // Get the control joints
    const int root_id = mj_name2id(model_.get(), mjOBJ_JOINT, baseName_);
    const int operator_id = mj_name2id(model_.get(), mjOBJ_JOINT, jointName_);
    rootQPos_ = model_->jnt_qposadr[root_id];
    jointQPos_ = model_->jnt_qposadr[operator_id];
}

void mc::MujocoClient::initializeGlfw()
{
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        throw std::runtime_error("Failed to initialize GLFW");
    }

    // Create a GLFW window
    GLFWwindow* raw_window = glfwCreateWindow(800, 600, "MuJoCo Demo", nullptr, nullptr);
    if (!raw_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    window_ = std::shared_ptr<GLFWwindow>(raw_window, [](GLFWwindow* window) {
            if (window) {
                glfwDestroyWindow(window);
            }
        });

    glfwMakeContextCurrent(window_.get());
    glEnable(GL_DEPTH_TEST);

    glfwSetWindowUserPointer(window_.get(), this);

    std::cout << "User pointer set to "<< glfwGetWindowUserPointer(window_.get()) << std::endl;


    // Register callbacks
    glfwSetMouseButtonCallback(window_.get(), mouse_button_callback);
    glfwSetCursorPosCallback(window_.get(), cursor_position_callback);
    glfwSetScrollCallback(window_.get(), scroll_callback);
}

void mc::MujocoClient::render() {
    // Get the framebuffer size
    int width, height;
    glfwGetFramebufferSize(window_.get(), &width, &height);

    // Set the OpenGL viewport
    glViewport(0, 0, width, height);

    mjrRect viewport = {0, 0, width, height};

    // Clear OpenGL buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Update MuJoCo scene
    mjv_updateScene(model_.get(), data_.get(), &opt_, nullptr, &cam_, mjCAT_ALL, &scene_);

    // Render the scene
    mjr_render(viewport, &scene_, &context_);

    // Swap buffers
    glfwSwapBuffers(window_.get());
}

void mc::MujocoClient::setConfig(const std::vector<double> &config) {
    // Set the pos[x,y,z] of the robot base
    data_->qpos[rootQPos_] = config[0];
    data_->qpos[rootQPos_+1] = config[1];
    data_->qpos[rootQPos_+2] = config[2];

    // Set the yaw of the robot base
    auto quad =rpyToQuaternion(0, 0, config[3]);
    data_->qpos[rootQPos_+3] = quad.w();
    data_->qpos[rootQPos_+4] = quad.x();
    data_->qpos[rootQPos_+5] = quad.y();
    data_->qpos[rootQPos_+6] = quad.z();

    // Set the joint angle of the operator
    data_->qpos[jointQPos_] = config[4];

    mj_fwdPosition(model_.get(), data_.get());
}

bool mc::MujocoClient::checkCollision(const std::vector<double> &config) {
    setConfig(config);
    mj_collision(model_.get(), data_.get());
    return data_->ncon > 0;
}

bool mc::MujocoClient::checkCollision() const
{
    mj_collision(model_.get(), data_.get());
    return data_->ncon > 0;
}

void mc::MujocoClient::mouse_button_callback_impl(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            isMousePressed_ = true;
            glfwGetCursorPos(window, &lastMouseX_, &lastMouseY_);
        } else if (action == GLFW_RELEASE) {
            isMousePressed_ = false;
        }
    }
}

void mc::MujocoClient::cursor_position_callback_impl(GLFWwindow* window, double xpos, double ypos) {
    if (isMousePressed_) {
        double dx = xpos - lastMouseX_;
        double dy = ypos - lastMouseY_;
        lastMouseX_ = xpos;
        lastMouseY_ = ypos;

        // Update camera angles
        cam_.azimuth += dx * 0.1; // Adjust rotation speed
        cam_.elevation -= dy * 0.1;

        // Clamp elevation to prevent flipping
        if (cam_.elevation > 90) cam_.elevation = 90;
        if (cam_.elevation < -90) cam_.elevation = -90;
    }
}

void mc::MujocoClient::scroll_callback_impl(GLFWwindow* window, double xoffset, double yoffset) {
    cam_.distance -= yoffset * 0.1; // Adjust zoom speed
    if (cam_.distance < 0.1) cam_.distance = 0.1; // Prevent distance from becoming too small
}





