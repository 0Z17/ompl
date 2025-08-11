#include "PlanningConfig.h"

namespace planning_config {

bool PlanningConfig::loadFromFile(const std::string& config_file) {
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        
        // Load files configuration
        auto files_node = config["files"];
        files.pcd_file = getRequired<std::string>(files_node, "pcd_file");
        files.model_file = getRequired<std::string>(files_node, "model_file");
        files.output_dir = getRequired<std::string>(files_node, "output_dir");
        
        // Load planning configuration
        auto planning_node = config["planning"];
        planning.type = getRequired<std::string>(planning_node, "type");
        planning.sample_num = getRequired<int>(planning_node, "sample_num");
        planning.planning_rounds = getRequired<int>(planning_node, "planning_rounds");
        planning.planning_timeout = getRequired<double>(planning_node, "planning_timeout");
        planning.atlas_timeout = getRequired<double>(planning_node, "atlas_timeout");
        planning.random_seed = getRequired<unsigned int>(planning_node, "random_seed");
        
        // Load trajectory configuration
        auto trajectory_node = config["trajectory"];
        trajectory.start_config = getRequired<std::vector<double>>(trajectory_node, "start_config");
        trajectory.goal_config = getRequired<std::vector<double>>(trajectory_node, "goal_config");
        
        // Load weights
        weights = getRequired<std::vector<double>>(config, "weights");
        
        // Load inverse kinematics configuration
        auto ik_node = config["inverse_kinematics"];
        inverse_kinematics.link_length = getRequired<double>(ik_node, "link_length");
        inverse_kinematics.clip_bound = getRequired<std::vector<double>>(ik_node, "clip_bound");
        
        // Load state bounds
        auto bounds_node = config["state_bounds"];
        state_bounds.x = getRequired<std::vector<double>>(bounds_node, "x");
        state_bounds.y = getRequired<std::vector<double>>(bounds_node, "y");
        state_bounds.z = getRequired<std::vector<double>>(bounds_node, "z");
        state_bounds.psi = getRequired<std::vector<double>>(bounds_node, "psi");
        state_bounds.theta = getRequired<std::vector<double>>(bounds_node, "theta");
        
        // Load parameter space configuration
        auto param_space_node = config["parameter_space"];
        parameter_space.dimensions = getRequired<int>(param_space_node, "dimensions");
        parameter_space.bounds_low = getRequired<double>(param_space_node, "bounds_low");
        parameter_space.bounds_high = getRequired<double>(param_space_node, "bounds_high");
        
        // Load state space configuration
        auto state_space_node = config["state_space"];
        state_space.dimensions = getRequired<int>(state_space_node, "dimensions");
        
        // Load surface constraint configuration
        auto constraint_node = config["surface_constraint"];
        surface_constraint.tolerance = getRequired<double>(constraint_node, "tolerance");
        
        // Load motion validation configuration
        auto motion_node = config["motion_validation"];
        motion_validation.cost_resolution_factor = getRequired<double>(motion_node, "cost_resolution_factor");
        
        // Load rendering configuration
        auto rendering_node = config["rendering"];
        rendering.enabled = getRequired<bool>(rendering_node, "enabled");
        rendering.render_frequency = getRequired<double>(rendering_node, "render_frequency");
        rendering.calculation_frequency = getRequired<double>(rendering_node, "calculation_frequency");
        
        // Load output configuration
        auto output_node = config["output"];
        output.save_surface_stl = getRequired<bool>(output_node, "save_surface_stl");
        output.surface_normal = getRequired<std::vector<double>>(output_node, "surface_normal");
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading config file: " << e.what() << std::endl;
        return false;
    }
}

void PlanningConfig::printConfig() const {
    std::cout << "=== Planning Configuration ===" << std::endl;
    std::cout << "Files:" << std::endl;
    std::cout << "  PCD File: " << files.pcd_file << std::endl;
    std::cout << "  Model File: " << files.model_file << std::endl;
    std::cout << "  Output Dir: " << files.output_dir << std::endl;
    
    std::cout << "Planning:" << std::endl;
    std::cout << "  Type: " << planning.type << std::endl;
    std::cout << "  Sample Num: " << planning.sample_num << std::endl;
    std::cout << "  Planning Rounds: " << planning.planning_rounds << std::endl;
    std::cout << "  Planning Timeout: " << planning.planning_timeout << std::endl;
    std::cout << "  Atlas Timeout: " << planning.atlas_timeout << std::endl;
    std::cout << "  Random Seed: " << planning.random_seed << std::endl;
    
    std::cout << "Trajectory:" << std::endl;
    std::cout << "  Start Config: [";
    for (size_t i = 0; i < trajectory.start_config.size(); ++i) {
        std::cout << trajectory.start_config[i];
        if (i < trajectory.start_config.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    std::cout << "  Goal Config: [";
    for (size_t i = 0; i < trajectory.goal_config.size(); ++i) {
        std::cout << trajectory.goal_config[i];
        if (i < trajectory.goal_config.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    std::cout << "Weights: [";
    for (size_t i = 0; i < weights.size(); ++i) {
        std::cout << weights[i];
        if (i < weights.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    std::cout << "Rendering: " << (rendering.enabled ? "Enabled" : "Disabled") << std::endl;
    std::cout << "==============================" << std::endl;
}

} // namespace planning_config