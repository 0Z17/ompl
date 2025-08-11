#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/Constraint.h>
#include <ompl/geometric/planners/fmt/PCSFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/EstimatePathLengthOptimizationObjective.h>
#include <ompl/base/objectives/WeightedPathLengthOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/AdaptiveDiscreteMotionValidator.h>
#include <ompl/util/Time.h>
#include <ompl/config.h>

#include "invkin.h"
#include "nurbs.h"
#include "mujoco_client.h"
#include "ConstrainedPlanningCommon.h"
#include "PlanningConfig.h"

#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <memory>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dp = dynamic_planning;
namespace sr = surface_reconstructor;
namespace mc = mujoco_client;
namespace pc = planning_config;

enum class PlanningType {
    FMT,
    PCSFMT,
    AtlasRRTstar,
    BundleBITstar
};

class PlanningSystem {
public:
    PlanningSystem(const std::string& config_file);
    ~PlanningSystem();
    
    bool initialize();
    void runPlanning();
    
private:
    // Configuration
    pc::PlanningConfig config_;
    
    // Core components
    std::unique_ptr<sr::Nurbs> nurbs_;
    std::unique_ptr<dp::InvKin> ik_;
    std::unique_ptr<mc::MujocoClient> client_; 
    
    // Planning results
    ob::PathPtr path_;
    ob::PathPtr state_path_;
    double planning_time_;
    std::vector<dynamic_planning::Vector5d> coll_config_;
    std::vector<std::vector<double>> constrained_path_data_;
    bool path_found_;
    
    // Helper methods
    PlanningType stringToPlanningType(const std::string& type_str);
    bool isStateValid(const ob::State *state);
    bool isStateValidForQ(const ob::State *state);
    bool isStateValidForAtlas(const ob::State *state);
    
    // Path analysis methods
    void getPathCsv(const ob::PathPtr &path, const std::string &filename);
    double getPathCost(const ob::PathPtr &path, const std::vector<double> &weight);
    double getOperatorMovement(const ob::PathPtr &path);
    void getPlanningData(const int idx, const ob::PathPtr &path, const double planning_time, 
                        const std::string &filename, bool isValid = true);
    
    // CSV reading utility
    std::vector<std::vector<double>> readCsv(const std::string& filename);
    
    // Planning methods
    void planWithType(PlanningType planning_type);
    void setupConstrainedPlanning(PlanningType planning_type);
    void setupUnconstrainedPlanning(PlanningType planning_type);
    void processResults(PlanningType planning_type, int idx);
    void renderResults(PlanningType planning_type, const std::vector<std::vector<double>>& data);
    
    // Surface constraint class
    class SurfaceConstraint : public ob::Constraint {
    public:
        SurfaceConstraint(const sr::Nurbs* nurbs, dp::InvKin* ik)
            : ob::Constraint(5, 3, 1e-3), nurbs_(nurbs), ik_(ik) {}
        
        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override {
            const auto peFull = ik_->qToQe(x);
            const Eigen::Vector3d pe = x.head<3>();
            double u, v;
            nurbs_->getClosestPoint(pe, u, v);
            Eigen::Vector3d surface_point;
            nurbs_->getPos(u, v, surface_point);
            auto qs = ik_->xToQ(u,v);
            const double distance = (pe - surface_point).norm();
            out[0] = distance;
            out[1] = qs[3] - x[3];
            out[2] = qs[4] - x[4];
        }
        
    private:
        const sr::Nurbs* nurbs_;
        dp::InvKin* ik_;
    };
};

PlanningSystem::PlanningSystem(const std::string& config_file) 
    : planning_time_(0.0), path_found_(false) {
    if (!config_.loadFromFile(config_file)) {
        throw std::runtime_error("Failed to load configuration file: " + config_file);
    }
}

PlanningSystem::~PlanningSystem() = default;

bool PlanningSystem::initialize() {
    try {
        // Initialize NURBS surface
        nurbs_ = std::make_unique<sr::Nurbs>(config_.files.pcd_file);
        
        // Initialize inverse kinematics
        ik_ = std::make_unique<dp::InvKin>(nurbs_.get());
        ik_->setLinkLength(config_.inverse_kinematics.link_length);
        ik_->setClipBound(config_.inverse_kinematics.clip_bound);
        
        // Initialize MuJoCo client
        client_ = std::make_unique<mc::MujocoClient>(config_.files.model_file.c_str());
        
        // Set random seed
        ompl::RNG::setSeed(config_.planning.random_seed);
        
        // Fit surface and save if configured
        Eigen::Vector3d normal(config_.output.surface_normal[0], 
            config_.output.surface_normal[1], 
            config_.output.surface_normal[2]);
        nurbs_->fitSurface(normal);
        if (config_.output.save_surface_stl) {
            std::string stl_path = config_.files.output_dir + "/surface.stl";
            nurbs_->saveSurfaceAsStl(stl_path);
        }
        
        config_.printConfig();
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Initialization failed: " << e.what() << std::endl;
        return false;
    }
}

PlanningType PlanningSystem::stringToPlanningType(const std::string& type_str) {
    if (type_str == "FMT") return PlanningType::FMT;
    if (type_str == "PCSFMT") return PlanningType::PCSFMT;
    if (type_str == "AtlasRRTstar") return PlanningType::AtlasRRTstar;
    if (type_str == "BundleBITstar") return PlanningType::BundleBITstar;
    throw std::runtime_error("Unknown planning type: " + type_str);
}

bool PlanningSystem::isStateValid(const ob::State *state) {
    auto u = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
    auto v = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
    auto q = ik_->xToQ(u, v);
    auto ret = client_->isCollision(std::vector<double>(q.data(), q.data() + q.size()));
    return !ret;
}

bool PlanningSystem::isStateValidForQ(const ob::State *state) {
    auto q = state->as<ob::RealVectorStateSpace::StateType>()->values;
    auto ret = client_->isCollision(std::vector<double>(q, q + 5));
    return !ret;
}

bool PlanningSystem::isStateValidForAtlas(const ob::State *state) {
    auto q = state->as<ob::ConstrainedStateSpace::StateType>()->getState()
             ->as<ob::RealVectorStateSpace::StateType>()->values;
    auto ret = client_->isCollision(std::vector<double>(q, q + 5));
    return !ret;
}

void PlanningSystem::getPathCsv(const ob::PathPtr &path, const std::string &filename) {
    std::ofstream file(filename);
    file << "X,Y,Z,Psi,Theta\n";
    
    for (const auto point : path->as<og::PathGeometric>()->getStates()) {
        const auto statePt = point->as<ob::RealVectorStateSpace::StateType>();
        file << statePt->values[0] << "," << statePt->values[1] << "," 
             << statePt->values[2] << "," << statePt->values[3] << "," 
             << statePt->values[4] << "\n";
    }
}

double PlanningSystem::getPathCost(const ob::PathPtr &path, const std::vector<double> &weight) {
    auto pathStates = path->as<og::PathGeometric>()->getStates();
    double cost = 0.0;
    
    for (auto it = pathStates.begin(); it != pathStates.end(); ++it) {
        const auto statePt = (*it)->as<ob::RealVectorStateSpace::StateType>();
        if (it != pathStates.begin()) {
            const auto prevStatePt = (*(it-1))->as<ob::RealVectorStateSpace::StateType>();
            cost += std::sqrt(
                std::pow(weight[0] * (statePt->values[0] - prevStatePt->values[0]), 2) +
                std::pow(weight[1] * (statePt->values[1] - prevStatePt->values[1]), 2) +
                std::pow(weight[2] * (statePt->values[2] - prevStatePt->values[2]), 2) +
                std::pow(weight[3] * (statePt->values[3] - prevStatePt->values[3]), 2) +
                std::pow(weight[4] * (statePt->values[4] - prevStatePt->values[4]), 2)
            );
        }
    }
    return cost;
}

double PlanningSystem::getOperatorMovement(const ob::PathPtr &path) {
    auto pathStates = path->as<og::PathGeometric>()->getStates();
    double movement = 0.0;
    
    for (auto it = pathStates.begin(); it != pathStates.end(); ++it) {
        const auto statePt = (*it)->as<ob::RealVectorStateSpace::StateType>();
        if (it != pathStates.begin()) {
            const auto prevStatePt = (*(it-1))->as<ob::RealVectorStateSpace::StateType>();
            movement += std::sqrt(std::pow(statePt->values[4] - prevStatePt->values[4], 2));
        }
    }
    return movement;
}

void PlanningSystem::getPlanningData(const int idx, const ob::PathPtr &path, 
                                   const double planning_time, const std::string &filename, bool isValid) {
    std::ofstream file(filename, std::ios::app);
    
    if (file.tellp() == 0) {
        file << "idx,planning_time,path_cost,operator_movement\n";
    }
    
    if (isValid) {
        file << idx << "," << planning_time << "," 
             << getPathCost(path, config_.weights) << "," 
             << getOperatorMovement(path) << "\n";
    } else {
        file << idx << ",NaN,NaN,NaN\n";
    }
}

std::vector<std::vector<double>> PlanningSystem::readCsv(const std::string& filename) {
    std::vector<std::vector<double>> data;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return data;
    }
    
    std::string line;
    bool header_skipped = false;
    
    while (std::getline(file, line)) {
        if (!header_skipped) {
            header_skipped = true;
            continue;
        }
        
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;
        
        while (std::getline(ss, value, ',')) {
            try {
                row.push_back(std::stod(value));
            } catch (const std::exception& e) {
                std::cerr << "Error converting value: " << value << " - " << e.what() << std::endl;
                row.clear();
                break;
            }
        }
        
        if (!row.empty()) {
            data.push_back(row);
        }
    }
    
    return data;
}

void PlanningSystem::runPlanning() {
    PlanningType planning_type = stringToPlanningType(config_.planning.type);
    
    for (int round = 0; round < config_.planning.planning_rounds; round++) {
        std::cout << "Planning round: " << (round + 1) << "/" << config_.planning.planning_rounds << std::endl;
        
        planWithType(planning_type);
        processResults(planning_type, round + 1);
        
        if (config_.rendering.enabled) {
            bool should_render = false;
            std::vector<std::vector<double>> data;
            
            if (planning_type == PlanningType::AtlasRRTstar || planning_type == PlanningType::BundleBITstar) {
                should_render = path_found_ && !constrained_path_data_.empty();
            } else {
                should_render = (path_ != nullptr && state_path_ != nullptr);
            }
            
            if (should_render) {
                renderResults(planning_type, data);
            }
        }
    }
}

void PlanningSystem::planWithType(PlanningType planning_type) {
    if (planning_type == PlanningType::AtlasRRTstar || planning_type == PlanningType::BundleBITstar) {
        setupConstrainedPlanning(planning_type);
    } else {
        setupUnconstrainedPlanning(planning_type);
    }
}

void PlanningSystem::setupUnconstrainedPlanning(PlanningType planning_type) {
    // Create parameter space
    auto space = std::make_shared<ob::RealVectorStateSpace>(config_.parameter_space.dimensions);
    ob::RealVectorBounds bounds(config_.parameter_space.dimensions);
    bounds.setLow(config_.parameter_space.bounds_low);
    bounds.setHigh(config_.parameter_space.bounds_high);
    space->setBounds(bounds);
    
    // Create state space
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(config_.state_space.dimensions);
    
    // Setup space information
    auto si = std::make_shared<ob::SpaceInformation>(space);
    auto stateSi = std::make_shared<ob::SpaceInformation>(stateSpace);
    
    si->setStateValidityChecker([this](const ob::State *state) {
        return this->isStateValid(state);
    });
    
    auto motionValidator = std::make_shared<ompl::base::AdaptiveDiscreteMotionValidator>(si);
    if (planning_type == PlanningType::PCSFMT) {
        si->setMotionValidator(motionValidator);
    }
    si->setup();
    
    // Setup motion validator cost resolution
    Eigen::Matrix<double, 5, 1> weightsVec;
    weightsVec << config_.weights[0], config_.weights[1], config_.weights[2], 
                  config_.weights[3], config_.weights[4];
    
    auto qStart = ik_->xToQ(config_.trajectory.start_config[0], config_.trajectory.start_config[1]);
    auto qGoal = ik_->xToQ(config_.trajectory.goal_config[0], config_.trajectory.goal_config[1]);
    double estimateResol = (qStart - qGoal).cwiseProduct(weightsVec).norm() * 
                          config_.motion_validation.cost_resolution_factor;
    motionValidator->setCostResolution(estimateResol);
    
    stateSi->setStateValidityChecker([this](const ob::State *state) {
        return this->isStateValidForQ(state);
    });
    
    state_path_ = std::make_shared<og::PathGeometric>(stateSi);
    
    // Setup optimization objective
    ob::OptimizationObjectivePtr opt;
    if (planning_type == PlanningType::PCSFMT) {
        opt = std::make_shared<ob::EstimatePathLengthOptimizationObjective>(si);
    } else if (planning_type == PlanningType::FMT) {
        opt = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    }
    
    // Setup start and goal states
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = config_.trajectory.start_config[0];
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = config_.trajectory.start_config[1];
    
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = config_.trajectory.goal_config[0];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = config_.trajectory.goal_config[1];
    
    // Create problem definition
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);
    pdef->setOptimizationObjective(opt);
    
    // Create planner
    ob::PlannerPtr planner;
    if (planning_type == PlanningType::PCSFMT) {
        planner = std::make_shared<og::PCSFMT>(si, stateSi, ik_.get());
        dynamic_cast<og::PCSFMT*>(planner.get())->setNumSamples(config_.planning.sample_num);
        dynamic_cast<og::PCSFMT*>(planner.get())->setWeights(config_.weights);
    } else if (planning_type == PlanningType::FMT) {
        planner = std::make_shared<og::FMT>(si);
        dynamic_cast<og::FMT*>(planner.get())->setNumSamples(config_.planning.sample_num);
    }
    
    planner->setProblemDefinition(pdef);
    planner->setup();
    
    // Solve
    ompl::time::point start_time = ompl::time::now();
    ob::PlannerStatus solved = planner->ob::Planner::solve(config_.planning.planning_timeout);
    planning_time_ = ompl::time::seconds(ompl::time::now() - start_time);
    
    std::cout << "Planning time: " << planning_time_ << " seconds" << std::endl;
    
    if (solved) {
        path_ = pdef->getSolutionPath();
        std::cout << "Found solution!" << std::endl;
        
        // Save planner data
        std::string planner_data_file = config_.files.output_dir + "/" + 
                                       config_.planning.type + "_planner_data.csv";
        if (planning_type == PlanningType::PCSFMT) {
            static_cast<og::PCSFMT*>(planner.get())->getPlannerDataCsv(planner_data_file);
        } else if (planning_type == PlanningType::FMT) {
            static_cast<og::FMT*>(planner.get())->getPlannerDataCsv(planner_data_file);
        }
        
        // Convert to state path
        ob::State *s = stateSi->allocState();
        for (auto point : path_->as<og::PathGeometric>()->getStates()) {
            auto u = point->as<ob::RealVectorStateSpace::StateType>()->values[0];
            auto v = point->as<ob::RealVectorStateSpace::StateType>()->values[1];
            auto q = ik_->xToQ(u, v);
            
            auto stateS = s->as<ob::RealVectorStateSpace::StateType>();
            stateS->values[0] = q(0);
            stateS->values[1] = q(1);
            stateS->values[2] = q(2);
            stateS->values[3] = q(3);
            stateS->values[4] = q(4);
            
            state_path_->as<og::PathGeometric>()->append(s->as<ob::State>());
        }
        
        // Smooth path
        og::PathSimplifier ps(stateSi);
        ps.smoothBSpline(*state_path_->as<og::PathGeometric>(), 3, 0.0005);
        
    } else {
        std::cout << "No solution found" << std::endl;
        path_ = nullptr;
    }
}

void PlanningSystem::setupConstrainedPlanning(PlanningType planning_type) {
    // Create constrained state space
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(5);
    
    // Set state bounds
    ob::RealVectorBounds stateBounds(5);
    stateBounds.setLow(0, config_.state_bounds.x[0]);
    stateBounds.setHigh(0, config_.state_bounds.x[1]);
    stateBounds.setLow(1, config_.state_bounds.y[0]);
    stateBounds.setHigh(1, config_.state_bounds.y[1]);
    stateBounds.setLow(2, config_.state_bounds.z[0]);
    stateBounds.setHigh(2, config_.state_bounds.z[1]);
    stateBounds.setLow(3, config_.state_bounds.psi[0]);
    stateBounds.setHigh(3, config_.state_bounds.psi[1]);
    stateBounds.setLow(4, config_.state_bounds.theta[0]);
    stateBounds.setHigh(4, config_.state_bounds.theta[1]);
    rvss->setBounds(stateBounds);
    
    // Create surface constraint
    auto constraint = std::make_shared<SurfaceConstraint>(nurbs_.get(), ik_.get());
    
    // Create constrained state spaces
    std::shared_ptr<ob::ConstrainedStateSpace> css;
    std::shared_ptr<ob::ConstrainedSpaceInformation> csi;
    
    if (planning_type == PlanningType::AtlasRRTstar) {
        css = std::make_shared<ob::AtlasStateSpace>(rvss, constraint);
        csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
    } else if (planning_type == PlanningType::BundleBITstar) {
        css = std::make_shared<ob::TangentBundleStateSpace>(rvss, constraint);
        csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
    }
    
    csi->setStateValidityChecker([this](const ob::State *state) {
        return this->isStateValidForAtlas(state);
    });
    csi->setup();
    
    // Create optimization objective
    Eigen::Matrix<double, 5, 1> costWeights;
    costWeights << config_.weights[0], config_.weights[1], config_.weights[2], 
                   config_.weights[3], config_.weights[4];
    auto opt = std::make_shared<ob::WeightedPathLengthOptimizationObjective>(csi, costWeights);
    
    // Setup start and goal states
    auto state_config_start = ik_->xToQ(config_.trajectory.start_config[0], config_.trajectory.start_config[1]);
    auto state_config_goal = ik_->xToQ(config_.trajectory.goal_config[0], config_.trajectory.goal_config[1]);
    
    ob::ScopedState<> state_start(css);
    ob::ScopedState<> state_goal(css);
    state_start->as<ob::ConstrainedStateSpace::StateType>()->copy(state_config_start);
    state_goal->as<ob::ConstrainedStateSpace::StateType>()->copy(state_config_goal);
    
    // Anchor charts for Atlas state space
    if (planning_type == PlanningType::AtlasRRTstar) {
        css->as<ob::AtlasStateSpace>()->anchorChart(state_start.get());
        css->as<ob::AtlasStateSpace>()->anchorChart(state_goal.get());
    } else if (planning_type == PlanningType::BundleBITstar) {
        css->as<ob::TangentBundleStateSpace>()->anchorChart(state_start.get());
        css->as<ob::TangentBundleStateSpace>()->anchorChart(state_goal.get());
    }
    
    // Create simple setup
    auto ss = std::make_shared<og::SimpleSetup>(csi);
    ss->setStartAndGoalStates(state_start, state_goal);
    ss->setOptimizationObjective(opt);
    
    // Create and configure planner
    ob::PlannerPtr planner;
    if (planning_type == PlanningType::AtlasRRTstar) {
        planner = std::make_shared<og::RRTstar>(csi);
    } else if (planning_type == PlanningType::BundleBITstar) {
        planner = std::make_shared<og::BITstar>(csi);
    }
    
    ss->setPlanner(planner);
    ss->setup();
    
    // Solve
    ompl::time::point start_time = ompl::time::now();
    ob::PlannerStatus solved = ss->solve(config_.planning.planning_timeout);
    planning_time_ = ompl::time::seconds(ompl::time::now() - start_time);
    
    std::cout << "Planning time: " << planning_time_ << " seconds" << std::endl;
    
    if (solved) {
        auto pathConstrained = ss->getSolutionPath();
        std::cout << "Found solution!" << std::endl;
        pathConstrained.print(std::cout);
        
        std::cout << "Interpolating path..." << std::endl;
        pathConstrained.interpolate();
        
        if (!pathConstrained.check()) {
            std::cout << "Warning: Interpolated path fails check!" << std::endl;
        }
        
        // Save path to CSV
        std::string output_file = config_.files.output_dir + "/state_path_" + config_.planning.type + ".csv";
        std::ofstream file(output_file);
        file << "X,Y,Z,Psi,Theta\n";
        
        for (auto point : pathConstrained.getStates()) {
            const Eigen::Map<Eigen::VectorXd> &point_data = *point->as<ob::ConstrainedStateSpace::StateType>();
            file << point_data[0] << "," << point_data[1] << "," << point_data[2] 
                 << "," << point_data[3] << "," << point_data[4] << "\n";
        }
        file.close();
        
        // Store constrained path for rendering
        constrained_path_data_.clear();
        for (auto point : pathConstrained.getStates()) {
            const Eigen::Map<Eigen::VectorXd> &point_data = *point->as<ob::ConstrainedStateSpace::StateType>();
            std::vector<double> state_vec = {point_data[0], point_data[1], point_data[2], 
                                           point_data[3], point_data[4]};
            constrained_path_data_.push_back(state_vec);
        }
        
        path_found_ = true;
        
    } else {
        std::cout << "No solution found" << std::endl;
        path_found_ = false;
    }
}

void PlanningSystem::processResults(PlanningType planning_type, int idx) {
    std::string data_file = config_.files.output_dir + "/planning_data_" + config_.planning.type + ".csv";
    
    if (planning_type == PlanningType::AtlasRRTstar || planning_type == PlanningType::BundleBITstar) {
        // For constrained planning, results are already saved in setupConstrainedPlanning
        getPlanningData(idx, nullptr, planning_time_, data_file, path_found_);
        return;
    }
    
    if (!path_ || !state_path_) {
        getPlanningData(idx, nullptr, planning_time_, data_file, false);
        return;
    }
    
    // Save state path for unconstrained planning
    std::string state_path_file = config_.files.output_dir + "/state_path_" + config_.planning.type + ".csv";
    getPathCsv(state_path_, state_path_file);
    
    // Save planning data
    getPlanningData(idx, state_path_, planning_time_, data_file, true);
}

void PlanningSystem::renderResults(PlanningType planning_type, const std::vector<std::vector<double>>& data) {
    if (!config_.rendering.enabled) return;
    
    const auto rate_render = std::chrono::milliseconds(
        static_cast<int>(1000 / config_.rendering.render_frequency));
    const auto rate_cal = std::chrono::milliseconds(
        static_cast<int>(1000 / config_.rendering.calculation_frequency));
    
    auto next_render = std::chrono::steady_clock::now();
    auto next_cal = std::chrono::steady_clock::now();
    
    int count = 0;
    
    while (!glfwWindowShouldClose(client_->getWindow())) {
        auto now = std::chrono::steady_clock::now();
        
        if (now > next_render) {
            client_->render();
            glfwPollEvents();
            next_render += rate_render;
        }
        
        if (now > next_cal) {
            count++;
            
            if (planning_type == PlanningType::AtlasRRTstar || 
                planning_type == PlanningType::BundleBITstar) {
                // Use constrained path data
                if (count > static_cast<int>(constrained_path_data_.size())) {
                    continue;
                }
                auto q = constrained_path_data_[count-1];
                client_->setConfig(q);
            } else {
                // Use unconstrained path data
                if (!state_path_) continue;
                auto states = state_path_->as<og::PathGeometric>()->getStates();
                if (count > static_cast<int>(states.size())) {
                    continue;
                }
                const auto point = states[count-1];
                const auto statePt = point->as<ob::RealVectorStateSpace::StateType>();
                std::vector<double> q(statePt->values, statePt->values + 5);
                client_->setConfig(q);
            }
            
            next_cal += rate_cal;
        }
    }
}

int main(int argc, char **argv) {
    std::string config_file = "/home/wsl/proj/my_ompl/demos/MyPlanners/config.yaml";
    
    if (argc > 1) {
        config_file = argv[1];
    }
    
    try {
        PlanningSystem planning_system(config_file);
        
        if (!planning_system.initialize()) {
            std::cerr << "Failed to initialize planning system" << std::endl;
            return -1;
        }
        
        planning_system.runPlanning();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}