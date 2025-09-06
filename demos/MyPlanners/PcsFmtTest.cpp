#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/fmt/PCSFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/EstimatePathLengthOptimizationObjective.h>
#include <ompl/base/objectives/WeightedPathLengthOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/AdaptiveDiscreteMotionValidator.h>
#include "invkin.h"
#include "nurbs.h"
#include <pcl/io/pcd_io.h>
#include <ompl/util/Time.h>
#include <ompl/config.h>
#include <iostream>
#include "mujoco_client.h"
#include <chrono>
#include <thread>

#include "ConstrainedPlanningCommon.h"
#include "../PlanarManipulator/PlanarManipulatorIKGoal.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dp = dynamic_planning;
namespace sr = surface_reconstructor;
namespace mc = mujoco_client;

enum PlanningType
{
    FMT,
    PCSFMT,
    AtlasRRTstar,
    BundleBITstar
};

 std::string pcdFile = "/home/wsl/proj/skyvortex_mujoco/assets/NURBS.pcd";
// std::string pcdFile = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/pointcloud_bridge1.pcd";
// std::string pcdFile = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/pointcloud_plane2.pcd";
// std::string pcdFile = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/pointcloud_cylinder.pcd";
// std::string pcdFile = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/pointcloud_EXP.pcd";
// std::string pcdFile = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/pointcloud_turbine.pcd";
// std::string pcdFile =  "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/surface_complex.pcd";
// std::string pcdFile =  "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/blade_segment.pcd";
// std::string pcdFile =  "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/surf.pcd";
// std::string pcdFile =  "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/scans_S2.pcd";
// std::string pcdFile =  "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/tube_pointcloud.pcd";
// std::string pcdFile =  "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/pointcloud.pcd";
//std::string pcdFile =  "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/surface_outdoor.pcd";
std::string modelFile = "/home/wsl/proj/skyvortex_mujoco/scene.xml";
const auto nurbs = new sr::Nurbs(pcdFile);
auto ik = new dp::InvKin(nurbs);
mc::MujocoClient client(modelFile.c_str());
std::vector<dynamic_planning::Vector5d> collConfig;
ob::PathPtr path;
ob::PathPtr statePath;
double planning_time = 0.0;
unsigned int num_sample = 0u;
// std::vector<double> weights = {1.0, 1.0, 1.0, 1.0, 6.0};
// std::vector<double> weights = {1.0, 1.0, 1.0, 1.5, 2.5};
// std::vector<double> weights = {1.0, 1.0, 1.0, 3.0, 6.0};
// std::vector<double> weights = {1.0, 1.0, 1.0, 1.5, 3.5};
std::vector<double> weights = {1.0, 1.0, 1.0, 3.0, 6.0};
 // std::vector<double> weights = {1.0, 1.0, 1.0, 1.5, 2.5};
// std::vector<double> weights = {1.0, 1.0, 1.0, 2, 3.5};
//std::vector<double> weights = {1.0, 1.0, 1.0, 6.5, 6.5};

bool isStateValid(const ob::State *state)
{
    // alway return true for test
    // @todo: implement a real state validity checking function

    auto u =state->as<ob::RealVectorStateSpace::StateType>()->values[0];
    auto v =state->as<ob::RealVectorStateSpace::StateType>()->values[1];
    auto q =ik->xToQ(u,v);
    auto ret = client.isCollision(std::vector<double>(q.data(), q.data() + q.size()));
    // OMPL_INFORM("cc result: %s", ret ? "true" : "false");
    // if (ret) collConfig.push_back(q);
    return !ret;
     // return true;
}

std::vector<std::vector<double>> read_csv(const std::string& filename) {
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
            continue; // skip header line
        }

        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {
            try {
                row.push_back(std::stod(value));
            } catch (const std::exception& e) {
                std::cerr << "Error converting value: " << value
                          << " - " << e.what() << std::endl;
                row.clear();
                break;
            }
        }

        if (!row.empty()) {
            data.push_back(row);
        }
    }

    file.close();
    return data;
}

bool isStateValidForQ(const ob::State *state)
{
    // alway return true for test
    // // @todo: implement a real state validity checking function
    //
    auto q =state->as<ob::RealVectorStateSpace::StateType>()->values;
    auto vec = std::vector<double>(q, q + 5);
    // OMPL_INFORM("check state: %f, %f, %f, %f, %f", vec[0], vec[1], vec[2], vec[3], vec[4]);
    auto ret = client.isCollision(std::vector<double>(q, q + 5));
    // OMPL_INFORM("cc result: %s", ret ? "true" : "false");
    // if (ret) collConfig.push_back(q);
    return !ret;
    // return true;
}

bool isStateValidForAtlas(const ob::State *state)
{
    // alway return true for test
    // // @todo: implement a real state validity checking function
    //
    auto q = state->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<ob::RealVectorStateSpace::StateType>()->values;
    auto vec = std::vector<double>(q, q + 5);
    // OMPL_INFORM("check state: %f, %f, %f, %f, %f", vec[0], vec[1], vec[2], vec[3], vec[4]);
    auto ret = client.isCollision(std::vector<double>(q, q + 5));
    // OMPL_INFORM("cc result: %s", ret ? "true" : "false");
    // if (ret) collConfig.push_back(q);
    return !ret;
     // return true;
}

void getPathCsv(const ob::PathPtr &path, const std::string &filename)
{
    std::ofstream file(filename);
    file << "X,Y,Z,Psi,Theta\n";

    for (const auto point : path->as<og::PathGeometric>()->getStates())
    {
        const auto statePt = point->as<ob::RealVectorStateSpace::StateType>();
        file << statePt->values[0]
        << "," << statePt->values[1]
        << "," << statePt->values[2]
        << "," << statePt->values[3]
        << "," << statePt->values[4]
        << "\n";
    }
}

double getPathCost(const ob::PathPtr &path, const std::vector<double> &weight)
{
    auto pathStates = path->as<og::PathGeometric>()->getStates();
    double cost = 0.0;
    for (auto it = pathStates.begin(); it != pathStates.end(); ++it)
    {
        const auto statePt = (*it)->as<ob::RealVectorStateSpace::StateType>();
        if (it != pathStates.begin())
        {
            const auto prevStatePt = (*(it-1))->as<ob::RealVectorStateSpace::StateType>();
            // std::cout << statePt->values[0] << " " << statePt->values[1] << " " << statePt->values[2] << " " << statePt->values[3] << " " << statePt->values[4] << std::endl;
            cost += std::sqrt(  std::pow(weight[0] * (statePt->values[0] - prevStatePt->values[0]), 2) +
                                std::pow(weight[1] * (statePt->values[1] - prevStatePt->values[1]), 2) +
                                std::pow(weight[2] * (statePt->values[2] - prevStatePt->values[2]), 2) +
                                std::pow(weight[3] * (statePt->values[3] - prevStatePt->values[3]), 2) +
                                std::pow(weight[4] * (statePt->values[4] - prevStatePt->values[4]), 2));
        }
    }

    return cost;
}

double getOperatorMovement(const ob::PathPtr &path)
{
    auto pathStates = path->as<og::PathGeometric>()->getStates();
    double movement = 0.0;
    for (auto it = pathStates.begin(); it != pathStates.end(); ++it)
    {
        const auto statePt = (*it)->as<ob::RealVectorStateSpace::StateType>();
        if (it != pathStates.begin())
        {
            const auto prevStatePt = (*(it-1))->as<ob::RealVectorStateSpace::StateType>();
            movement += std::sqrt(std::pow(statePt->values[4] - prevStatePt->values[4], 2));
        }
    }
    return movement;
}

void getPlanningData(const int idx, const ob::PathPtr &path, const double planning_time, const std::string &filename, bool isValid = true)
{
    std::ofstream file(filename, std::ios::app);
    // if the file is empty, write the header
    if (file.tellp() == 0)
    {
        file << "idx,planning_time,path_cost,operator_movement,num_sample, isValid\n";
    }
    if (isValid)
    {
        file << idx << ","
             << planning_time << ","
             << getPathCost(path, weights) << ","
             << getOperatorMovement(path) << ","
             << num_sample << ","
             << "1\n";
    }
    else
    {
        file << idx << ","
        << planning_time << ","
        << getPathCost(path, weights) << ","
        << getOperatorMovement(path) << ","
        << num_sample << ","
        << "0\n";
        // file << idx << ","
        //      << "NaN" << ","
        //      << "NaN" << ","
        //      << "NaN" << ","
        //      << num_sample << "\n";
    }
    file.close();
}


void plan(PlanningType planning_type)
{
    ik->setLinkLength(0.96);
	//     std::vector<double> clipBound = {0.2, 0.8, 0.2, 0.8};
    std::vector<double> clipBound = {0.1, 0.9, 0.1, 0.9};
    ik->setClipBound(clipBound);
    // ik->setLinkLength(1.0);
//    unsigned int seed = 114514;
//    ompl::RNG::setSeed(seed);
    Eigen::Vector3d pe;
    pe << 1.08513, 1.12521, 0.99239;
    double u, v;
    nurbs->getClosestPoint(pe, u, v);

    std::cout << pe[0] << "," << pe[1] << "," << pe[2] << "\n";
    std::cout << u << "," << v << "\n";
    std::cout << "//////////////////////////////////////" << "\n";

    auto qMin = ik->xToQ(0,0);
    auto qMax = ik->xToQ(1,1);
    std::cout << "qMin = " << qMin << "\n";
    std::cout << "qMax = " << qMax << "\n";
    std::cout << "//////////////////////////////////////" << "\n";

     // std::vector<double> start_config = {0.2, 0.8};
     // std::vector<double> goal_config = {0.8, 0.2};

    std::vector<double> start_config = {0.2, 0.2};
    std::vector<double> goal_config = {0.8, 0.8};


    // std::vector<double> start_config = {0.4, 0.1};
    // std::vector<double> goal_config = {0.7, 0.6};

     // std::vector<double> start_config = {0.2, 0.2};
     // std::vector<double> goal_config = {0.8, 0.8};

    // std::vector<double> start_config = {0.2, 0.2};
    // std::vector<double> goal_config = {0.5, 0.5};

    // std::vector<double> start_config = {0.5, 0.5};
    // std::vector<double> goal_config = {0.8, 0.2};

    /* The second trajectory for wind turbine surface inspection */
//     std::vector<double> start_config = {0.2, 0.1};
//     std::vector<double> goal_config = {0.6, 0.7};

    /* The second trajectory for wind turbine surface inspection */
    // std::vector<double> start_config = {0.6, 0.7};
    // std::vector<double> goal_config = {0.8, 0.1};

    /* The trajectory for complex surface inspection */
    // std::vector<double> start_config = {0.4, 0.3};
    // std::vector<double> goal_config = {0.6, 0.5};

    /* The trajectory for outdoor surface inspection */
//    std::vector<double> start_config = {0.4, 0.3};
//    std::vector<double> goal_config = {0.6, 0.7};

//    std::vector<double> start_config = {0.6, 0.7};
//    std::vector<double> goal_config = {0.9, 0.1};
//    std::vector<double> goal_config = {0.771109, 0.257148};

    // std::vector<double> start_config = {0.2, 0.1};
    // std::vector<double> goal_config = {0.6, 0.7};

    // std::vector<double> start_config = {0.6, 0.7};
    // std::vector<double> goal_config = {0.9, 0.1};

    // std::vector<double> start_config = {0.4, 0.3};
    // std::vector<double> goal_config = {0.5, 0.555};

    std::vector<double> start_config = {0.4, 0.3};
    std::vector<double> goal_config = {0.6, 0.5};


    /* params for the planner */
    uint paramDimensionsNum = 2;
    uint stateDimensionsNum = 5;
    // std::string pcdFile = "/home/wsl/proj/pcl/test/milk.pcd";

    auto space(std::make_shared<ob::RealVectorStateSpace>(paramDimensionsNum));
    auto stateSpace(std::make_shared<ob::RealVectorStateSpace>(stateDimensionsNum));

    // Atlas configuration Space
    class SurfaceConstraint : public ob::Constraint
    {
    public:
         SurfaceConstraint(const sr::Nurbs* nurbs) : ompl::base::Constraint(5, 3, 3e-1), nurbs_(nurbs) {};
//        SurfaceConstraint(const sr::Nurbs* nurbs) : ompl::base::Constraint(5, 3, 3e-3), nurbs_(nurbs) {};

        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
        {
            const auto peFull = ik->qToQe(x);
            // std::cout << x << std::endl;
            const Eigen::Vector3d pe = peFull.head<3>();
            double u, v;
            nurbs_->getClosestPoint(pe, u, v);
            Eigen::Vector3d surface_point;
            nurbs_->getPos(u, v, surface_point);
            auto qs = ik->xToQ(u,v);
            const double distance = (pe-surface_point).norm();
            out[0] = distance;
            out[1] = qs[3] - x[3];
            out[2] = qs[4] - x[4];

        }

        // void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
        // {
        //     auto J = ik->getJacobian(x);
        //     const auto peFull = ik->qToQe(x);
        //     const Eigen::Vector3d pe = peFull.head<3>();
        //     double u, v;
        //     nurbs_->getClosestPoint(pe, u, v);
        //     Eigen::Vector3d normal;
        //     nurbs_->getNormal(u, v, normal);
        //     auto jacbCon = normal.transpose() * J.block(0, 0, 3, 5);
        //     out.block(0, 0, 1, 5) = jacbCon;
        //     out.block(1, 0, 2, 5) = J.block(1, 0, 2, 5);
        // }

    private:
        const sr::Nurbs* nurbs_;
    };


    auto rvss = std::make_shared<ob::RealVectorStateSpace>(5);

    ob::RealVectorBounds stateBounds(5);
//    std::vector<double> bound_x{0.0, 4.0}, bound_y{-4.0, 3.0}, bound_z{0.0, 3.5},
//                        bound_psi{-M_PI/2, M_PI/2}, bound_theta{-M_PI/2, M_PI/2};

    // std::vector<double> bound_x{0.5, 2.0}, bound_y{-1.9, 1.9}, bound_z{0.0, 4},
    //                 bound_psi{-M_PI/2, M_PI/2}, bound_theta{-M_PI/2, M_PI/2};

    /** For blade segment */
    // std::vector<double> bound_x{4.0, 8.5}, bound_y{13.0, 28.0}, bound_z{60.0, 72.0},
    //                 bound_psi{-1*M_PI/6, 1*M_PI/6}, bound_theta{-M_PI/2, M_PI/2};

    /** For complex surface */
    // std::vector<double> bound_x{1.0, 3.6}, bound_y{0.0, 3.5}, bound_z{0.0, 4},
    //             bound_psi{-M_PI/2, M_PI/2}, bound_theta{-M_PI/2, M_PI/2};

    // /** For NURBS surface */
    // std::vector<double> bound_x{0.55, 0.75}, bound_y{-1.6, 2.35}, bound_z{0.5, 3.5},
    // bound_psi{-M_PI/2, M_PI/2}, bound_theta{-M_PI/2, M_PI/2};

    /** For expe Blade surface*/
    // std::vector<double> bound_x{0.7, 1.80}, bound_y{-1.5, 1.6}, bound_z{0.2, 2.1},
    //                     bound_psi{-M_PI/2, M_PI/2}, bound_theta{-M_PI/2, M_PI/2};

    /** For the complex surface */
    std::vector<double> bound_x{1.8, 2.2}, bound_y{0.9, 2.1}, bound_z{0.3, 2.6},
                    bound_psi{-1*M_PI/6, 1*M_PI/6}, bound_theta{-M_PI/2, M_PI/2};

    stateBounds.setLow(0, bound_x[0]);
    stateBounds.setHigh(0, bound_x[1]);
    stateBounds.setLow(1, bound_y[0]);
    stateBounds.setHigh(1, bound_y[1]);
    stateBounds.setLow(2, bound_z[0]);
    stateBounds.setHigh(2, bound_z[1]);
    stateBounds.setLow(3, bound_psi[0]);
    stateBounds.setHigh(3, bound_psi[1]);
    stateBounds.setLow(4, bound_theta[0]);
    stateBounds.setHigh(4, bound_theta[1]);
    rvss->setBounds(stateBounds);

    auto constraint = std::make_shared<SurfaceConstraint>(nurbs);
    auto css = std::make_shared<ob::AtlasStateSpace>(rvss, constraint);
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
    csi->setStateValidityChecker(isStateValidForAtlas);
    csi->setup();

    // for BundleBITstar
    auto cssBundle = std::make_shared<ob::TangentBundleStateSpace>(rvss, constraint);
    auto csiBundle = std::make_shared<ob::ConstrainedSpaceInformation>(cssBundle);
    csiBundle->setStateValidityChecker(isStateValidForAtlas);
    csiBundle->setup();


    ob::RealVectorBounds bounds(paramDimensionsNum);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // si->setStateValidityCheckingResolution(0.02);
    std::cout<< "space valid check resolution: " << si->getStateValidityCheckingResolution() << std::endl;
    // create a space information instance for the state spa
    auto stateSi(std::make_shared<ob::SpaceInformation>(stateSpace));

    // set the state validity checking for the param space
    si->setStateValidityChecker(isStateValid);
    auto motionValidator = std::make_shared<ompl::base::AdaptiveDiscreteMotionValidator>(si);

    if (planning_type == PCSFMT) si->setMotionValidator(motionValidator);
    si->setup();

    Eigen::Matrix<double, 5, 1> weightsVec;
    weightsVec << weights[0] , weights[1] , weights[2],  weights[3] , weights[4];

    auto qStart = ik->xToQ(start_config[0], start_config[1]);
    auto qGoal = ik->xToQ(goal_config[0], goal_config[1]);

    double estimateResol = (qStart - qGoal).cwiseProduct(weightsVec).norm() * 0.05;
    // double estimateResol = (qStart - qGoal).cwiseProduct(weightsVec).norm() * 0.2;

    motionValidator->setCostResolution(estimateResol);

    stateSi->setStateValidityChecker(isStateValidForQ);
    // stateSi->setup();

    statePath = std::make_shared<og::PathGeometric>(stateSi);

    ob::OptimizationObjectivePtr opt;

    if (planning_type == PCSFMT)
    // set the optimal objective
    {
        opt = std::make_shared<ob::EstimatePathLengthOptimizationObjective>(si);
    }
    else if (planning_type == FMT)
    {
        opt = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    }
    else if (planning_type == AtlasRRTstar)
    {
        // opt = std::make_shared<ob::PathLengthOptimizationObjective>(csi);
        Eigen::Matrix<double, 5, 1> costWeights;
        costWeights << weights[0] , weights[1] , weights[2],  weights[3] , weights[4];
        opt = std::make_shared<ob::WeightedPathLengthOptimizationObjective>(csi, costWeights);
    }
    else if (planning_type == BundleBITstar)
    {
        // opt = std::make_shared<ob::PathLengthOptimizationObjective>(csi);
        Eigen::Matrix<double, 5, 1> costWeights;
        costWeights << weights[0] , weights[1] , weights[2],  weights[3] , weights[4];
        opt = std::make_shared<ob::WeightedPathLengthOptimizationObjective>(csiBundle, costWeights);
    }


    auto ss = std::make_shared<og::SimpleSetup>(csi);


    // setting for the AtlasRRTstar planner
    auto state_config_ls = ik->xToQ(start_config[0], start_config[1]);
    std::cout << "state_config_ls: " << state_config_ls.transpose() << std::endl;
    double cu, cv;
    auto qe = ik->qToQe(state_config_ls);
    auto qs = ik->xToQs(start_config[0], start_config[1]);
    std::cout << "qs: " << qs.transpose() << std::endl;
    std::cout << "qe: " << qe.transpose() << std::endl;
    nurbs->getClosestPoint(state_config_ls.head<3>(), cu, cv);
    auto surface_point = ik->xToQs(cu, cv);
    std::cout << "surface_point: " << surface_point.transpose() << std::endl;
    std::cout << "closest point: " << cu << ", " << cv << std::endl;
    auto goal_config_ls = ik->xToQ(goal_config[0], goal_config[1]);

    ob::ScopedState<> state_start(css);
    ob::ScopedState<> state_goal(css);
    state_start->as<ob::ConstrainedStateSpace::StateType>()->copy(state_config_ls);
    state_goal->as<ob::ConstrainedStateSpace::StateType>()->copy(goal_config_ls);

    css->as<ob::AtlasStateSpace>()->anchorChart(state_start.get());
    css->as<ob::AtlasStateSpace>()->anchorChart(state_goal.get());

    ss->setStartAndGoalStates(state_start, state_goal);
    ss->setOptimizationObjective(opt);

    auto plannerCon = std::make_shared<og::RRTstar>(csi);
    ss->setPlanner(plannerCon);
    ss->setup();
    auto maxMotion = plannerCon->getRange();
    // plannerCon->setRange(0.4);
    std::cout << maxMotion << std::endl;
    // plannerCon->getSpecs().approximateSolutions = false;

    // setting for the BundleBITstar planner
    auto ssBundle = std::make_shared<og::SimpleSetup>(csiBundle);
    ob::ScopedState<> state_startBundle(cssBundle);
    ob::ScopedState<> state_goalBundle(cssBundle);
    state_startBundle->as<ob::ConstrainedStateSpace::StateType>()->copy(state_config_ls);
    state_goalBundle->as<ob::ConstrainedStateSpace::StateType>()->copy(goal_config_ls);

    cssBundle->as<ob::AtlasStateSpace>()->anchorChart(state_startBundle.get());
    cssBundle->as<ob::AtlasStateSpace>()->anchorChart(state_goalBundle.get());
    ssBundle->setStartAndGoalStates(state_startBundle, state_goalBundle);
    ssBundle->setOptimizationObjective(opt);

    auto plannerCon2 = std::make_shared<og::BITstar>(csiBundle);
    ssBundle->setPlanner(plannerCon2);
    ssBundle->setup();





    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    // create a random start state
    ob::ScopedState<> start(space);
    // start.random();
    // std::vector<double> start_config = {0.2, 0.1};
    // std::vector<double> goal_config = {0.8, 0.9};

    // std::vector<double> start_config = {0.9, 0.5};
    // std::vector<double> goal_config = {0.1, 0.1};



    // For experiment
    // std::vector<double> start_config = {0.3, 0.7};
    // std::vector<double> goal_config = {0.7, 0.3};
    //
    // std::vector<double> start_config = {0.3, 0.7};
    // std::vector<double> goal_config = {0.3, 0.3};

    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_config[0];
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_config[1];

    // create a random goal state
    ob::ScopedState<> goal(space);
    // goal.random();
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_config[0];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_config[1];



    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // set the optimization objective
    pdef->setOptimizationObjective(opt);

    // load the inverse kinematics solver
    // const auto nurbs = new sr::Nurbs(pcdFile);

    // auto ik = new dp::InvKin(nurbs);

    // create a planner for the defined space
    ob::PlannerPtr planner;
//     int sampleNum = 40000;
    int sampleNum = 5000;
    // int sampleNum = 4000;
    if (planning_type == PCSFMT)
    {
        planner = std::make_shared<og::PCSFMT>(si, stateSi, ik);
        dynamic_cast<og::PCSFMT*>(planner.get())->setNumSamples(sampleNum);
        dynamic_cast<og::PCSFMT*>(planner.get())->setWeights(weights);
        num_sample = sampleNum;
    }
    else if (planning_type == FMT)
    {
        planner = std::make_shared<og::FMT>(si);
        dynamic_cast<og::FMT*>(planner.get())->setNumSamples(sampleNum);
        // dynamic_cast<og::FMT*>(planner.get())->setNearestK(false);
        num_sample = sampleNum;
    }


    if (planning_type != AtlasRRTstar && planning_type != BundleBITstar)
    {
        // set the problem we are trying to solve for the planner
        planner->setProblemDefinition(pdef);

        // perform setup steps for the planner
        planner->setup();


        // print the settings for this space
        si->printSettings(std::cout);

        // print the problem settings
        pdef->print(std::cout);
    }


    ompl::time::point start_time = ompl::time::now();
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved;
    if (planning_type == AtlasRRTstar)
    {
        // solved = ss->solve(1.5);
        plannerCon->setKNearest(false);
        // solved = ss->solve(5.0);
        solved = ss->solve(2.0);
        std::cout << "sample_attempt: " << num_sample << std::endl;
        // solved = ss->solve(60.0);
        // solved = ss->solve(120.0);
        num_sample = plannerCon->getNumSamples();
    }
    else if (planning_type == BundleBITstar)
    {
        solved = ssBundle->solve(5.0);
    }
    else
    {
        solved = planner->ob::Planner::solve(20.0);
    }
    planning_time = ompl::time::seconds(ompl::time::now() - start_time);
    OMPL_INFORM("Planning time: %.3f seconds", planning_time);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        if (planning_type == PCSFMT)
        {
            static_cast<og::PCSFMT*>(planner.get())->getPlannerDataCsv("/home/wsl/proj/my_ompl/demos/MyPlanners/"
                                                                       "test_output/PCSFMT_planner_data.csv");
        }
        else if (planning_type == FMT)
        {
            static_cast<og::FMT*>(planner.get())->getPlannerDataCsv("/home/wsl/proj/my_ompl/demos/MyPlanners/"
                                                                   "test_output/FMT_planner_data.csv");
        }

        // ob::PlannerData data(si);
        // planner->getPlannerData(data);

        // print the path to screen
        if (planning_type != AtlasRRTstar && planning_type != BundleBITstar)
        {
            path->print(std::cout);
        }
    }
    else
    {
        if (planning_type == PCSFMT)
        {
            static_cast<og::PCSFMT*>(planner.get())->getPlannerDataCsv("/home/wsl/proj/my_ompl/demos/MyPlanners/"
                                                                       "test_output/PCSFMT_planner_data.csv");
        }
        else if (planning_type == FMT)
        {
            static_cast<og::FMT*>(planner.get())->getPlannerDataCsv("/home/wsl/proj/my_ompl/demos/MyPlanners/"
                                                                   "test_output/FMT_planner_data.csv");
        }
        std::cout << "No solution found" << std::endl;
    }

    if (planning_type != AtlasRRTstar && planning_type != BundleBITstar)
    {
        // ob::ScopedState<> s(stateSi);
        ob::State *s = stateSi->allocState();
        //  // Postprocess
        for (auto point : path->as<og::PathGeometric>()->getStates())
        {
            auto u = point->as<ob::RealVectorStateSpace::StateType>()->values[0];
            auto v = point->as<ob::RealVectorStateSpace::StateType>()->values[1];
            auto q = ik->xToQ(u,v);
            // ob::ScopedState<> s(stateSi);
            auto stateS = s->as<ob::RealVectorStateSpace::StateType>();
            stateS->values[0] = q(0);
            stateS->values[1] = q(1);
            stateS->values[2] = q(2);
            // if (q(3) < 0) q(3) += 2*M_PI;
            stateS->values[3] = q(3);
            stateS->values[4] = q(4);

            OMPL_INFORM("state %f, %f, %f, %f, %f", stateS->values[0], stateS->values[1], stateS->values[2],
                stateS->values[3], stateS->values[4]);
            statePath->as<og::PathGeometric>()->append(s->as<ob::State>());
        }

        og::PathSimplifier ps(stateSi);
        ps.smoothBSpline(*statePath->as<og::PathGeometric>(), 3, 0.0005);
    }
    else if (planning_type == BundleBITstar)
    {
        auto pathBundle = ssBundle->getSolutionPath();
        pathBundle.print(std::cout);

        OMPL_INFORM("Interpolating path...");
        pathBundle.interpolate();

        if (!pathBundle.check())
            OMPL_WARN("Interpolated simplified path fails check!");

        std::ofstream file("/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_BundleBITstar.csv");
        file << "X,Y,Z,Psi,Theta\n";

        int idxP = 0;

        for (auto point : pathBundle.getStates())
        {
            ob::State *s = csi->allocState();
            const Eigen::Map<Eigen::VectorXd> &point_data = *point->as<ob::ConstrainedStateSpace::StateType>();
            csi->copyState(s, point);
            const Eigen::Map<Eigen::VectorXd> &stateS = *s->as<ob::ConstrainedStateSpace::StateType>();


            auto isValid = stateSi->checkMotion(pathBundle.getState(idxP),pathBundle.getState(idxP+1));

            file << point_data[0]
            << "," << point_data[1]
            << "," << point_data[2]
            << "," << point_data[3]
            << "," << point_data[4]
            << "\n";

            OMPL_INFORM("state %f, %f, %f, %f, %f", stateS[0], stateS[1], stateS[2],
                                                    stateS[3], stateS[4]);
            // statePath->as<og::PathGeometric>()->append(s->as<ob::State>());
        }

        // og::PathSimplifier ps(csi);
        // ps.smoothBSpline(*statePath->as<og::PathGeometric>(), 3, 0.0005);

    }
    else
    {

        auto pathAtlas = ss->getSolutionPath();
        pathAtlas.print(std::cout);


        OMPL_INFORM("Interpolating path...");
        pathAtlas.interpolate();

        if (!pathAtlas.check())
            OMPL_WARN("Interpolated simplified path fails check!");

        std::ofstream file("/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_AtlasRRTstar.csv");
        file << "X,Y,Z,Psi,Theta\n";


        for (auto point : pathAtlas.getStates())
        {
            ob::State *s = csi->allocState();
            const Eigen::Map<Eigen::VectorXd> &point_data = *point->as<ob::ConstrainedStateSpace::StateType>();
            csi->copyState(s, point);
            const Eigen::Map<Eigen::VectorXd> &stateS = *s->as<ob::ConstrainedStateSpace::StateType>();

            file << point_data[0]
            << "," << point_data[1]
            << "," << point_data[2]
            << "," << point_data[3]
            << "," << point_data[4]
            << "\n";

            OMPL_INFORM("state %f, %f, %f, %f, %f", stateS[0], stateS[1], stateS[2],
                                                    stateS[3], stateS[4]);
            // statePath->as<og::PathGeometric>()->append(s->as<ob::State>());
        }

        og::PathSimplifier ps(csi);
        ps.smoothBSpline(*statePath->as<og::PathGeometric>(), 3, 0.0005);

    }


}

int main(int argc, char **argv)
{
    int idx = 0;
    // int planningRound = 200;
    int planningRound = 100;
    // int planningRound = 1;

     bool isRenderResult = false;
    // bool isRenderResult = true;
    // PlanningType planning_type = PCSFMT;
    // PlanningType planning_type = FMT;
     PlanningType planning_type = AtlasRRTstar;
    // PlanningType planning_type = BundleBITstar;
    // glfwMakeContextCurrent(nullptr);
    nurbs->fitSurface(Eigen::Vector3d::UnitZ());
     // nurbs->fitSurface();
    nurbs->saveSurfaceAsStl("/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/surface_EXP.stl");

    dp::Vector6d qs = ik->xToQs(0.7,0.7);
    std::cout << qs << std::endl;

    dp::Vector6d qe = ik->xToQs(0.4,0.3);
    std::cout << qe << std::endl;

    for (int round =  0; round < planningRound; round++)
    {

        idx ++;
        plan(planning_type);
        int count = 0;

        std::vector<std::vector<double>> data;

        if (planning_type == AtlasRRTstar)
        {
            data = read_csv("/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_AtlasRRTstar.csv");
        }
        if (planning_type == BundleBITstar)
        {
            data = read_csv("/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_BundleBITstar.csv");
        }

        auto stateSpace(std::make_shared<ob::RealVectorStateSpace>(5));
        auto stateSi(std::make_shared<ob::SpaceInformation>(stateSpace));

        for (auto elem : data)
        {
            auto s = stateSi->allocState();
            for (int i = 0; i < 5; i++)
            {
                // std::cout << "elem " << elem[i] << " " << std::endl;
                s->as<ob::RealVectorStateSpace::StateType>()->values[i] = elem[i];
            }

            statePath->as<og::PathGeometric>()->append(s->as<ob::State>());
        }

        if (planning_type == PCSFMT)
        {
            getPathCsv(statePath, "/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_PCSFMT.csv");
            getPlanningData(idx, statePath, planning_time, "/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/planning_data_PCSFMT.csv");
        }
        else if (planning_type == FMT)
        {
            getPathCsv(statePath, "/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_FMT.csv");
            getPlanningData(idx, statePath, planning_time, "/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/planning_data_FMT.csv");
        }
        else if (planning_type == AtlasRRTstar)
        {
            // getPathCsv(statePath, "/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_AtlasRRTstar.csv");
            const ob::State *lastState = statePath->as<og::PathGeometric>()->getStates().back();
            dp::Vector5d lastConfig;
            lastConfig << lastState->as<ob::RealVectorStateSpace::StateType>()->values[0],
            lastState->as<ob::RealVectorStateSpace::StateType>()->values[1],
            lastState->as<ob::RealVectorStateSpace::StateType>()->values[2],
            lastState->as<ob::RealVectorStateSpace::StateType>()->values[3],
            lastState->as<ob::RealVectorStateSpace::StateType>()->values[4];

            auto goalConfig = ik->xToQ(0.6, 0.5);

            bool isValid = true;
            if ((goalConfig - lastConfig).norm() > 0.15 )
            {
                isValid = false;
                getPlanningData(idx, statePath, planning_time, "/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/planning_data_AtlasRRTstar.csv", isValid);
            }
            else
            {
                getPlanningData(idx, statePath, planning_time, "/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/planning_data_AtlasRRTstar.csv", isValid);
                // break;
            }

        }
        else if (planning_type == BundleBITstar)
        {
            // getPathCsv(statePath, "/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_AtlasRRTstar.csv");
            const ob::State *lastState = statePath->as<og::PathGeometric>()->getStates().back();
            dp::Vector5d lastConfig;
            lastConfig << lastState->as<ob::RealVectorStateSpace::StateType>()->values[0],
            lastState->as<ob::RealVectorStateSpace::StateType>()->values[1],
            lastState->as<ob::RealVectorStateSpace::StateType>()->values[2],
            lastState->as<ob::RealVectorStateSpace::StateType>()->values[3],
            lastState->as<ob::RealVectorStateSpace::StateType>()->values[4];

            auto goalConfig = ik->xToQ(0.8, 0.8);

            bool isValid = true;
            if ((goalConfig - lastConfig).norm() > 0.3 )
            {
                isValid = false;
            }
            else
            {
                getPlanningData(idx, statePath, planning_time, "/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/planning_data_BundleBITstar.csv", isValid);
                // break;
            }

        }
        // if (getPathCost(statePath, weights) < 3.00)
        // {
        //     isRenderResult = true;
        //     std::cout << "Path cost: " << getPathCost(statePath, weights) << std::endl;
        //     std::cout << "Path Cost: " << getOperatorMovement(statePath)*180/M_PI << std::endl;
        // }

        if (isRenderResult)
        {
            constexpr double frq_render = 100;
            // constexpr double frq_cal = 20;
            // constexpr double frq_cal = 20;
            constexpr double frq_cal = 5;
            constexpr auto rate_render = std::chrono::milliseconds(static_cast<int>(1000 / frq_render));
            constexpr auto rate_cal = std::chrono::milliseconds(static_cast<int>(1000 / frq_cal));

            auto nex_render = std::chrono::steady_clock::now();
            auto nex_cal = std::chrono::steady_clock::now();
            auto states = statePath->as<og::PathGeometric>()->getStates();

            while (!glfwWindowShouldClose(client.getWindow()))
            {
                auto now = std::chrono::steady_clock::now();
                // rendering loop
                if (now > nex_render)
                {
                    client.render();
                    glfwPollEvents();
                    nex_render += rate_render;
                }
                if (now > nex_cal)
                {
                    // planning loop
                    count++;
                    if (count > states.size())
                    {
                        continue;
                    }
                    if (planning_type != AtlasRRTstar)
                    {
                        const auto point = states[count-1];
                        const auto statePt = point->as<ob::RealVectorStateSpace::StateType>();
                        auto q = std::vector<double>(statePt->values, statePt->values + 5);
                        client.setConfig(std::vector<double>(statePt->values, statePt->values + 5));
                        nex_cal += rate_cal;
                    }
                    else
                    {
                        if (count > data.size())
                        {
                            continue;
                        }
                        auto q = data[count-1];
                        client.setConfig(q);
                        nex_cal += rate_cal;

                        // const auto point = states[count-1];
                        // // const auto statePt = point->as<ob::RealVectorStateSpace::StateType>();
                        // const auto statePt = point->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<ob::RealVectorStateSpace::StateType>();
                        // auto q = std::vector<double>(statePt->values, statePt->values + 5);
                        // std::cout<< q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << std::endl;
                        // client.setConfig(std::vector<double>(statePt->values, statePt->values + 5));
                        // nex_cal += rate_cal;
                    }

                }
            }
        }

    }

    // print the elems in the collConfig
    // for (auto elem : collConfig)
    // {
    //     // std::cout << elem.transpose() << std::endl;
    //     client.setConfig(std::vector<double>(elem.data(), elem.data() + elem.size()));
    //     client.render();
    //     glfwPollEvents();
    // }

    return 0;
}