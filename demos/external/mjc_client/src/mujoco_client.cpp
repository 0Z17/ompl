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

    scene_.maxgeom = model_->ngeom + 50;
    mjv_makeScene(model_.get(), &scene_, scene_.maxgeom);
    mjr_makeContext(model_.get(), &context_, mjFONTSCALE_150);

    // Set default camera position
    cam_.lookat[0] = camPoint_[0];
    cam_.lookat[1] = camPoint_[1];
    cam_.lookat[2] = camPoint_[2];
    cam_.distance = camDis_;
    cam_.azimuth = camAzimuth_;
    cam_.elevation = camElevation_;

    std::cout << "Model loaded: " << model_path_ << std::endl;

    // Get the control joints
    const int root_id = mj_name2id(model_.get(), mjOBJ_JOINT, baseJointName_);
    const int operator_id = mj_name2id(model_.get(), mjOBJ_JOINT, jointName_);
    baseId_ = mj_name2id(model_.get(), mjOBJ_BODY, baseBodyName_);
    rootQPos_ = model_->jnt_qposadr[root_id];
    jointQPos_ = model_->jnt_qposadr[operator_id];
    for (int i = 0; i < motorNames_.size(); i++) {
        motorIDs_.push_back(mj_name2id(model_.get(), mjOBJ_ACTUATOR, motorNames_[i]));
    }

    // // Get the sensor IDs info
    // sGyro_ = getSensorInfo("body_gyro");
    // sAcc_ = getSensorInfo("body_acc");
    // sQuat_ = getSensorInfo("body_quat");
    // sPos_ = getSensorInfo("body_pos");

    // // Set the wrench mapping
    // forwardForceMap_
    // <<  -0.249999,  0.499998,       -0.249999,  -0.249999,  0.499998,       -0.25,
    //     0.433009,   0,              -0.433009,  0.433009,   0,              -0.433013,
    //     0.866028,   0.866027,       0.866028,   0.866028,   0.866027,       0.866025,
    //     -0.375922,  -0.751848,      -0.375927,  0.375927,   0.751848,       0.375924,
    //     0.651114,   2.97481e-7,     -0.651117,  -0.651117,  2.97481e-7,     0.651119,
    //     0.258747,   -0.258742,      0.258744,   -0.258744,  0.258742,       -0.258741;
    //
    // inverseForceMap_
    // << -0.333334,   0.577356,       0.192449,   -0.221675,  0.383955,       0.644138,
    //     0.666668,   1.25305e-6,     0.192451,   -0.443352,  2.23507e-6,     -0.644141,
    //     -0.333332,  -0.577354,      0.192448,   -0.221675,  -0.383958,      0.644139,
    //     -0.333336,  0.577352,       0.192451,   0.221676,   -0.383953,      -0.644141,
    //     0.666671,   -2.68952e-6,    0.192449,   0.443353,   -1.27978e-6,    0.644139,
    //     -0.333335,  -0.577354,      0.19245,    0.221674,   0.383956,       -0.644136;


    // @DEBUG
    // std::cout << "Force mapping matrix" << std::endl;

    // for (int i = 0; i < model_->nq; i++) {
    //     if (std::isnan(data_->qpos[i]) || std::isinf(data_->qpos[i])) {
    //         std::cerr << "Invalid qpos[" << i << "] detected" << std::endl;
    //         data_->qpos[i] = 0.0;
    //     }
    // }


    // // Set the joint offset

    // setFT({0, 0, 20, 0, 0, 0});
    // for (int i = 0; i < 10; i++) {
    //     for (int j = 0; j < model_->nq; j++){
    //         std::cout << "qpos[" << j << "]: " << data_->qpos[j] << std::endl;
    //     }
    //     std::cout << "//////////////////////////////" << std::endl;
    //     // mj_step(model_.get(), data_.get());
    //     step();
    //     sleep(0.1);
    //     render();
    //     glfwPollEvents();
    // }
    // auto n =getBaseFT();
}

// void mc::MujocoClient::drawArrow(const Eigen::Vector3d & from,
//                               const Eigen::Vector3d & to,
//                               double shaft_diam)
// {
//     std::vector<float> color = {1.0,  0.2, 0.2, 0.8};
//
//     if (scene_.ngeom >= scene_.maxgeom)
//     {
//         return;
//     }
//     scene_.ngeom += 1;
//     mjvGeom* geom = &scene_.geoms[scene_.ngeom-1];
//     auto type = mjGEOM_ARROW;
//     Eigen::Vector3d dir_ = (to - from);
//     double size[3] = {shaft_diam, shaft_diam, 2 * dir_.norm()};
//     // Eigen::Matrix3d orientation = Eigen::Quaterniond::FromTwoVectors(dir_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
//
//     // mjv_initGeom(geom, type, size, from.data(), orientation.data(), color.data());
// }

// void mc::MujocoClient::showBaseAcc()
// {
//     Eigen::Vector3d acc = getBaseRotation() * getBaseAcc();
//     acc(2) = acc(2) - 9.81;
//     drawArrow(getBasePos(), getBasePos() + acc*0.1);
// }

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
    // showBaseAcc();

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
    data_->qpos[jointQPos_] = config[4] + jointOffset_;

    mj_fwdPosition(model_.get(), data_.get());
}

// void mc::MujocoClient::setFT(const std::vector<double> &ft) const
// {
//     const std::vector<double> thrust = inverseMapFT(ft);
//     // printf("thrust: %f %f %f %f %f %f\n", thrust[0], thrust[1], thrust[2], thrust[3], thrust[4], thrust[5]);
//     data_->ctrl[motorIDs_[0]] = thrust[0];
//     data_->ctrl[motorIDs_[1]] = thrust[1];
//     data_->ctrl[motorIDs_[2]] = thrust[2];
//     data_->ctrl[motorIDs_[3]] = thrust[3];
//     data_->ctrl[motorIDs_[4]] = thrust[4];
//     data_->ctrl[motorIDs_[5]] = thrust[5];
//     mj_step(model_.get(), data_.get());
// }

// std::vector<double> mc::MujocoClient::inverseMapFT(const std::vector<double> &ft) const
// {
//     const auto ft_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(ft.data());
//     const auto thrust = inverseForceMap_ * ft_;
//     return std::vector<double>{thrust[0], thrust[1], thrust[2], thrust[3], thrust[4], thrust[5]};
// }

// std::vector<double> mc::MujocoClient::getBaseFT() const
// {
//     // @TODO: get this data from sensor
// }

mc::SensorInfo mc::MujocoClient::getSensorInfo(const char* sensor_name) const
{
    // Get the sensor ID
    const int sensor_id = mj_name2id(model_.get(), mjOBJ_SENSOR, sensor_name);
    return {model_->sensor_adr[sensor_id], model_->sensor_dim[sensor_id]};
}

Eigen::Vector3d mc::MujocoClient::getBaseAngVel() const
{
    auto sensorData = data_->sensordata + sGyro_.adr;
    return {sensorData[0], sensorData[1], sensorData[2]};
}

Eigen::Vector3d mc::MujocoClient::getBaseAcc() const
{
    auto sensorData = data_->sensordata + sAcc_.adr;
    return {sensorData[0], sensorData[1], sensorData[2]};
}

Eigen::Vector3d mc::MujocoClient::getBasePos() const
{
    auto sensorData = data_->sensordata + sPos_.adr;
    return {sensorData[0], sensorData[1], sensorData[2]};
}

Eigen::Quaternion<double> mc::MujocoClient::getBaseQuat() const
{
    auto sensorData = data_->sensordata + sQuat_.adr;
    Eigen::Quaternion<double> quat(sensorData[0], sensorData[1], sensorData[2], sensorData[3]);
    quat.normalize();
    return quat;
}

Eigen::Matrix3d mc::MujocoClient::getBaseRotation() const
{
    const auto quat = getBaseQuat();
    return quat.toRotationMatrix();
}


bool mc::MujocoClient::isCollision(const std::vector<double> &config) {
    setConfig(config);
    mj_collision(model_.get(), data_.get());
    return data_->ncon > 0;
}

bool mc::MujocoClient::isCollision() const
{
    mj_collision(model_.get(), data_.get());
    return data_->ncon > 0;
}

void mc::MujocoClient::printCollisionInfo() const
{
    std::cout << "Collision info:" << std::endl;
    for (int j = 0; j < data_->ncon; j++) {
        const mjContact& contact = data_->contact[j];
        int geom1_id = contact.geom1;
        int geom2_id = contact.geom2;
        const char* geom1_name = mj_id2name(model_.get(), mjOBJ_GEOM, geom1_id);
        const char* geom2_name = mj_id2name(model_.get(), mjOBJ_GEOM, geom2_id);
        std::cout << "Contact: geom1=" << (geom1_name ? geom1_name : std::to_string(geom1_id).c_str())
        << " geom2=" << (geom2_name ? geom2_name : std::to_string(geom2_id).c_str())
        << " dist=" << contact.dist << std::endl;
    }

}

// void mc::MujocoClient::printSensorInfo() const
// {
//     std::cout << "Base Ang Vel: \n" << getBaseAngVel().transpose() << std::endl;
//     std::cout << "Base Acc: \n" << getBaseAcc().transpose() << std::endl;
//     std::cout << "Base Pos: \n" << getBasePos().transpose() << std::endl;
//     std::cout << "Base Quat: \n" << getBaseQuat().coeffs().transpose() << std::endl;
//     std::cout << "Base Rotation: \n" << getBaseRotation() << std::endl;
// }

void mc::MujocoClient::mouse_button_callback_impl(GLFWwindow* window, int button, int action, int mods) {
    buttonLeft_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    buttonMiddle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    buttonRight_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastMouseX_, &lastMouseY_);
}

void mc::MujocoClient::cursor_position_callback_impl(GLFWwindow* window, double xpos, double ypos) {
    if (!buttonLeft_ && !buttonMiddle_ && !buttonRight_) {
        return;
    }


    const double dx = xpos - lastMouseX_;
    const double dy = ypos - lastMouseY_;
    lastMouseX_ = xpos;
    lastMouseY_ = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (buttonRight_) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (buttonLeft_) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }
    mjv_moveCamera(model_.get(), action, dx/height, dy/height, &scene_, &cam_);
}

void mc::MujocoClient::scroll_callback_impl(GLFWwindow* window, double xoffset, double yoffset) {
    cam_.distance -= yoffset * 0.1; // Adjust zoom speed
    if (cam_.distance < 0.1) cam_.distance = 0.1; // Prevent distance from becoming too small
}





