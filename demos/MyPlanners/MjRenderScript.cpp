#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include "mujoco_client.h"

namespace mc = mujoco_client;

std::string getEnvVar(const std::string& var, const std::string& defaultValue) {
    const char* value = std::getenv(var.c_str());
    return value ? std::string(value) : defaultValue;
}

std::vector<std::vector<double>> readTrajectory(const std::string& filename) {
    std::vector<std::vector<double>> trajectory;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return trajectory;
    }

    std::string line;
    bool header_skipped = false;

    while (std::getline(file, line)) {
        if (!header_skipped) {
            header_skipped = true;
            continue; // skip header line
        }

        std::vector<double> config;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {
            try {
                config.push_back(std::stod(value));
            } catch (const std::exception& e) {
                std::cerr << "Error converting value: " << value
                          << " - " << e.what() << std::endl;
                config.clear();
                break;
            }
        }

        if (!config.empty()) {
            trajectory.push_back(config);
        }
    }

    file.close();
    return trajectory;
}

int main(int argc, char **argv) {
    // Get paths from environment variables
    std::string modelFile = getEnvVar("MODEL_FILE", "/home/wsl/proj/skyvortex_mujoco/scene.xml");
    std::string trajectoryFile = getEnvVar("TRAJECTORY_FILE", "/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/path.csv");
    
    // Initialize MujocoClient
    mc::MujocoClient client(modelFile.c_str());
    // 注意：构造函数中已经调用了 initializeGlfw()
    
    // Read trajectory from file
    std::vector<std::vector<double>> trajectory = readTrajectory(trajectoryFile);
    if (trajectory.empty()) {
        std::cerr << "No trajectory data found in file: " << trajectoryFile << std::endl;
        return 1;
    }
    
    std::cout << "Loaded trajectory with " << trajectory.size() << " waypoints." << std::endl;
    
    // Rendering and simulation parameters
    constexpr double frq_render = 100;
    constexpr double frq_cal = 50;
    constexpr auto rate_render = std::chrono::milliseconds(static_cast<int>(1000 / frq_render));
    constexpr auto rate_cal = std::chrono::milliseconds(static_cast<int>(1000 / frq_cal));

    auto nex_render = std::chrono::steady_clock::now();
    auto nex_cal = std::chrono::steady_clock::now();
    size_t count = 0;
    size_t trajectorySize = trajectory.size();
    
    // Main simulation loop
    while (!glfwWindowShouldClose(client.getWindow())) {
        auto now = std::chrono::steady_clock::now();
        
        // Rendering loop
        if (now > nex_render) {
            client.render();
            glfwPollEvents();
            nex_render += rate_render;
        }
        
        // Simulation loop - update robot config
        if (now > nex_cal) {
            // Get next config from trajectory
            size_t idx = count % trajectorySize;
            const auto& config = trajectory[idx];
            
            // Set config to robot
            client.setConfig(config);
            
            // Increment counter
            count++;
            nex_cal += rate_cal;
        }
    }
    
    return 0;
}
