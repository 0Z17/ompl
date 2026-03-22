#pragma once

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <stdexcept>

namespace planning_config {

struct Files {
    std::string pcd_file;
    std::string model_file;
    std::string output_dir;
};

struct Planning {
    std::string type;
    int sample_num;
    int planning_rounds;
    double planning_timeout;
    double atlas_timeout;
    unsigned int random_seed;
};

struct Trajectory {
    std::vector<std::vector<double>> waypoints;  // 多路径点序列
};

struct InverseKinematics {
    double link_length;
    std::vector<double> clip_bound;
};

struct StateBounds {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> psi;
    std::vector<double> theta;
};

struct ParameterSpace {
    int dimensions;
    double bounds_low;
    double bounds_high;
};

struct StateSpace {
    int dimensions;
};

struct SurfaceConstraint {
    double tolerance;
};

struct MotionValidation {
    double cost_resolution_factor;
};

struct BSpline {
    int steps;
    double dt;
};

struct Rendering {
    bool enabled;
    double render_frequency;
    double calculation_frequency;
};

struct Output {
    bool save_surface_stl;
    std::vector<double> surface_normal;
    bool export_visualization;
    int surface_grid_n;
};

class PlanningConfig {
public:
    Files files;
    Planning planning;
    Trajectory trajectory;
    std::vector<double> weights;
    InverseKinematics inverse_kinematics;
    StateBounds state_bounds;
    ParameterSpace parameter_space;
    StateSpace state_space;
    SurfaceConstraint surface_constraint;
    MotionValidation motion_validation;
    BSpline bspline;
    Rendering rendering;
    Output output;

    bool loadFromFile(const std::string& config_file);
    void printConfig() const;

private:
    template<typename T>
    T getRequired(const YAML::Node& node, const std::string& key) const {
        if (!node[key]) {
            throw std::runtime_error("Missing required config key: " + key);
        }
        return node[key].as<T>();
    }
    
    template<typename T>
    T getOptional(const YAML::Node& node, const std::string& key, const T& default_value) const {
        if (!node[key]) {
            return default_value;
        }
        return node[key].as<T>();
    }
};

} // namespace planning_config