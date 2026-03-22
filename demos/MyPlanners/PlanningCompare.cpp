// ============================================================
//  PlanningCompare.cpp  —  多规划器对比测试程序
//  支持 PCSFMT / FMT / AtlasRRTstar
//  通过 yaml 配置文件加载所有参数
//  用法: demo_PlanningCompare <config.yaml>
// ============================================================

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/fmt/PCSFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/EstimatePathLengthOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/WeightedPathLengthOptimizationObjective.h>
#include <ompl/base/AdaptiveDiscreteMotionValidator.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/util/Time.h>
#include <ompl/util/RandomNumbers.h>
#include "invkin.h"
#include "nurbs.h"
#include "mujoco_client.h"
#include "PlanningConfig.h"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <chrono>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dp = dynamic_planning;
namespace sr = surface_reconstructor;
namespace mc = mujoco_client;
namespace pc = planning_config;

// ============================================================
//  全局配置（从 yaml 加载）
// ============================================================
static pc::PlanningConfig g_cfg;

enum PlannerType { PLANNER_PCSFMT, PLANNER_FMT, PLANNER_ATLASRRTSTAR };

static PlannerType g_plannerType = PLANNER_PCSFMT;

static sr::Nurbs        *g_nurbs  = nullptr;
static dp::InvKin       *g_ik     = nullptr;
static mc::MujocoClient *g_client = nullptr;

static ob::PathPtr   g_path;
static ob::PathPtr   g_statePath;
static double        g_planningTime = 0.0;
static unsigned int  g_numSample    = 0u;

// ---- 用于复用采样的全局对象 ----
static ob::SpaceInformationPtr      g_reuseSi;
static ob::SpaceInformationPtr      g_reuseStateSi;
static std::shared_ptr<og::PCSFMT>  g_reusePlanner;
static bool                         g_samplesInitialized = false;

// ---- 碰撞检测（参数空间）----
bool isStateValid(const ob::State *state)
{
    return true;
}

// ---- 碰撞检测（关节空间）----
bool isStateValidForQ(const ob::State *state)
{
    return true;
}

// ---- 碰撞检测（Atlas 约束空间）----
bool isStateValidForAtlas(const ob::State *state)
{
    return true;
}

// ---- 路径代价计算 ----
double getPathCost(const ob::PathPtr &path)
{
    const auto &W = g_cfg.weights;
    auto pathStates = path->as<og::PathGeometric>()->getStates();
    double cost = 0.0;
    for (auto it = pathStates.begin(); it != pathStates.end(); ++it)
    {
        if (it == pathStates.begin()) continue;
        const auto cur  = (*it)->as<ob::RealVectorStateSpace::StateType>();
        const auto prev = (*(it-1))->as<ob::RealVectorStateSpace::StateType>();
        double sum = 0.0;
        for (int i = 0; i < 5; ++i)
            sum += std::pow(W[i] * (cur->values[i] - prev->values[i]), 2);
        cost += std::sqrt(sum);
    }
    return cost;
}

// ---- 机械臂运动量（Theta 轴）----
double getOperatorMovement(const ob::PathPtr &path)
{
    auto pathStates = path->as<og::PathGeometric>()->getStates();
    double movement = 0.0;
    for (auto it = pathStates.begin(); it != pathStates.end(); ++it)
    {
        if (it == pathStates.begin()) continue;
        const auto cur  = (*it)->as<ob::RealVectorStateSpace::StateType>();
        const auto prev = (*(it-1))->as<ob::RealVectorStateSpace::StateType>();
        movement += std::abs(cur->values[4] - prev->values[4]);
    }
    return movement;
}

// 1. 导出 NURBS 曲面离散点
void saveSurfaceGridCsv(const std::string &filename)
{
    const int N = g_cfg.output.surface_grid_n;
    std::ofstream file(filename);
    file << "u,v,x,y,z\n";
    for (int i = 0; i <= N; ++i)
    {
        double u = static_cast<double>(i) / N;
        for (int j = 0; j <= N; ++j)
        {
            double v = static_cast<double>(j) / N;
            Eigen::Vector3d pos;
            if (g_nurbs->getPos(u, v, pos) == 0)
                file << u << "," << v << "," << pos[0] << "," << pos[1] << "," << pos[2] << "\n";
        }
    }
}

// 2. 导出途径点的 xyz 位置
void saveWaypointsCsv(const std::string &filename)
{
    std::ofstream file(filename);
    file << "u,v,x,y,z\n";
    for (const auto &wp : g_cfg.trajectory.waypoints)
    {
        const auto qs = g_ik->xToQs(wp[0], wp[1]);
        file << wp[0] << "," << wp[1] << ","
             << qs[0] << "," << qs[1] << "," << qs[2] << "\n";
    }
}

// 3. 导出规划路径上每个配置的末端位置和法向量
void savePathContactCsv(const ob::PathPtr &path, const std::string &filename)
{
    std::ofstream file(filename);
    file << "x,y,z,nx,ny,nz\n";
    auto states = path->as<og::PathGeometric>()->getStates();
    for (const auto *s : states)
    {
        const auto *rv = s->as<ob::RealVectorStateSpace::StateType>();
        Eigen::VectorXd q(5);
        for (int i = 0; i < 5; ++i) q[i] = rv->values[i];
        const auto qe = g_ik->qToQe(q);
        const Eigen::Vector3d pe = qe.head<3>();

        double u, v;
        g_nurbs->getClosestPoint(pe, u, v);
        Eigen::Vector3d normal;
        g_nurbs->getNormal(u, v, normal);

        file << pe[0] << "," << pe[1] << "," << pe[2] << ","
             << normal[0] << "," << normal[1] << "," << normal[2] << "\n";
    }
}

// ---- 将路径输出为 CSV ----
void savePathCsv(const ob::PathPtr &path, const std::string &filename)
{
    std::ofstream file(filename);
    file << "X,Y,Z,Psi,Theta\n";
    for (const auto point : path->as<og::PathGeometric>()->getStates())
    {
        const auto s = point->as<ob::RealVectorStateSpace::StateType>();
        file << s->values[0] << "," << s->values[1] << "," << s->values[2]
             << "," << s->values[3] << "," << s->values[4] << "\n";
    }
}

// ---- 将规划统计数据追加到 CSV ----
void savePlanningData(int idx, const ob::PathPtr &path, double planTime, const std::string &filename)
{
    std::ofstream file(filename, std::ios::app);
    if (file.tellp() == 0)
        file << "idx,planning_time,path_cost,operator_movement,num_sample,isValid\n";

    file << idx << ","
         << planTime << ","
         << getPathCost(path) << ","
         << getOperatorMovement(path) << ","
         << g_numSample << ","
         << "1\n";
}

// ---- SurfaceConstraint 类定义（用于 AtlasRRTstar）----
class SurfaceConstraint : public ob::Constraint
{
public:
    SurfaceConstraint(const sr::Nurbs* nurbs)
        : ob::Constraint(5, 3, g_cfg.surface_constraint.tolerance), nurbs_(nurbs) {}

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        const auto peFull = g_ik->qToQe(x);
        const Eigen::Vector3d pe = peFull.head<3>();
        double u, v;
        nurbs_->getClosestPoint(pe, u, v);
        Eigen::Vector3d surface_point;
        nurbs_->getPos(u, v, surface_point);
        auto qs = g_ik->xToQ(u, v);
        const double distance = (pe - surface_point).norm();
        out[0] = distance / 0.01;
        out[1] = (qs[3] - x[3]) / 0.1;
        out[2] = (qs[4] - x[4]) / 0.1;
    }

private:
    const sr::Nurbs* nurbs_;
};

// ---- 核心规划函数：规划单段 startCfg -> goalCfg，返回关节空间路径 ----
std::shared_ptr<og::PathGeometric> planSegment(const std::vector<double> &startCfg,
                                               const std::vector<double> &goalCfg,
                                               int segIdx)
{
    const auto &W      = g_cfg.weights;
    const uint paramDim = 2;
    const uint stateDim = 5;

    g_ik->setLinkLength(g_cfg.inverse_kinematics.link_length);
    g_ik->setClipBound(g_cfg.inverse_kinematics.clip_bound);

    ob::SpaceInformationPtr si;
    ob::SpaceInformationPtr stateSi;
    std::shared_ptr<og::PCSFMT> pcsfmtPlanner;

    // AtlasRRTstar 使用约束空间
    if (g_plannerType == PLANNER_ATLASRRTSTAR)
    {
        OMPL_INFORM("Segment %d: Using AtlasRRTstar planner", segIdx);

        auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(stateDim);
        ob::RealVectorBounds stateBounds(stateDim);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(g_cfg.files.pcd_file, *cloud) == -1)
        {
            OMPL_ERROR("Failed to load point cloud file: %s", g_cfg.files.pcd_file.c_str());
            return nullptr;
        }

        Eigen::Vector3d minPt(std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max());
        Eigen::Vector3d maxPt(std::numeric_limits<double>::lowest(),
                              std::numeric_limits<double>::lowest(),
                              std::numeric_limits<double>::lowest());

        for (const auto& point : cloud->points)
        {
            minPt[0] = std::min(minPt[0], static_cast<double>(point.x));
            minPt[1] = std::min(minPt[1], static_cast<double>(point.y));
            minPt[2] = std::min(minPt[2], static_cast<double>(point.z));
            maxPt[0] = std::max(maxPt[0], static_cast<double>(point.x));
            maxPt[1] = std::max(maxPt[1], static_cast<double>(point.y));
            maxPt[2] = std::max(maxPt[2], static_cast<double>(point.z));
        }

        double offset = g_cfg.inverse_kinematics.link_length;
        std::vector<double> bound_x{minPt[0] - offset, maxPt[0] + offset};
        std::vector<double> bound_y{minPt[1] - 0.5 * offset, maxPt[1] + 0.5 * offset};
        std::vector<double> bound_z{minPt[2] - 0.5 * offset, maxPt[2] + 0.5 * offset};
        const auto &sb = g_cfg.state_bounds;

        stateBounds.setLow(0, bound_x[0]);   stateBounds.setHigh(0, bound_x[1]);
        stateBounds.setLow(1, bound_y[0]);   stateBounds.setHigh(1, bound_y[1]);
        stateBounds.setLow(2, bound_z[0]);   stateBounds.setHigh(2, bound_z[1]);
        stateBounds.setLow(3, sb.psi[0]);    stateBounds.setHigh(3, sb.psi[1]);
        stateBounds.setLow(4, sb.theta[0]);  stateBounds.setHigh(4, sb.theta[1]);
        stateSpace->setBounds(stateBounds);

        auto constraint = std::make_shared<SurfaceConstraint>(g_nurbs);
        auto css = std::make_shared<ob::AtlasStateSpace>(stateSpace, constraint);
        css->setRho(0.15);
        css->setEpsilon(0.05);
        css->setAlpha(M_PI / 4);
        auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
        csi->setStateValidityChecker(isStateValidForAtlas);
        csi->setup();

        auto qStart = g_ik->xToQ(startCfg[0], startCfg[1]);
        auto qGoal  = g_ik->xToQ(goalCfg[0],  goalCfg[1]);

        ob::ScopedState<> state_start(css);
        ob::ScopedState<> state_goal(css);
        state_start->as<ob::ConstrainedStateSpace::StateType>()->copy(qStart);
        state_goal->as<ob::ConstrainedStateSpace::StateType>()->copy(qGoal);

        if (!constraint->project(state_start.get()))
            OMPL_WARN("Failed to project start state onto constraint manifold!");
        if (!constraint->project(state_goal.get()))
            OMPL_WARN("Failed to project goal state onto constraint manifold!");

        css->as<ob::AtlasStateSpace>()->anchorChart(state_start.get());
        css->as<ob::AtlasStateSpace>()->anchorChart(state_goal.get());

        auto ss = std::make_shared<og::SimpleSetup>(csi);
        ss->setStartAndGoalStates(state_start, state_goal);

        Eigen::Matrix<double, 5, 1> costWeights;
        costWeights << W[0], W[1], W[2], W[3], W[4];
        auto opt = std::make_shared<ob::WeightedPathLengthOptimizationObjective>(csi, costWeights);
        ss->setOptimizationObjective(opt);

        auto planner = std::make_shared<og::RRTstar>(csi);
        planner->setKNearest(false);
        planner->setRange(0.2);
        ss->setPlanner(planner);
        ss->setup();

        ompl::time::point t0 = ompl::time::now();
        ob::PlannerStatus solved = ss->solve(g_cfg.planning.planning_timeout);
        g_planningTime += ompl::time::seconds(ompl::time::now() - t0);

        if (!solved)
        {
            std::cout << "Segment " << segIdx << ": No solution found" << std::endl;
            return nullptr;
        }

        ss->simplifySolution(0.05);
        auto pathAtlas = ss->getSolutionPath();
        pathAtlas.interpolate();

        auto resultStateSi = std::make_shared<ob::SpaceInformation>(
            std::make_shared<ob::RealVectorStateSpace>(stateDim));
        auto segStatePath = std::make_shared<og::PathGeometric>(resultStateSi);

        for (auto point : pathAtlas.getStates())
        {
            const Eigen::Map<Eigen::VectorXd> &stateData = *point->as<ob::ConstrainedStateSpace::StateType>();
            ob::State *s = resultStateSi->allocState();
            auto stateS = s->as<ob::RealVectorStateSpace::StateType>();
            for (int i = 0; i < 5; ++i)
                stateS->values[i] = stateData[i];
            segStatePath->append(s);
            resultStateSi->freeState(s);
        }

        return segStatePath;
    }

    // 第一次调用：初始化并采样
    if (!g_samplesInitialized && g_plannerType == PLANNER_PCSFMT)
    {
        OMPL_INFORM("Segment %d: Initializing samples and neighborhoods (first time)", segIdx);

        auto space      = std::make_shared<ob::RealVectorStateSpace>(paramDim);
        auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(stateDim);

        ob::RealVectorBounds bounds(paramDim);
        bounds.setLow(g_cfg.parameter_space.bounds_low);
        bounds.setHigh(g_cfg.parameter_space.bounds_high);
        space->setBounds(bounds);

        si      = std::make_shared<ob::SpaceInformation>(space);
        stateSi = std::make_shared<ob::SpaceInformation>(stateSpace);

        si->setStateValidityChecker(isStateValid);
        stateSi->setStateValidityChecker(isStateValidForQ);

        auto motionValidator = std::make_shared<ob::AdaptiveDiscreteMotionValidator>(si);
        Eigen::Matrix<double, 5, 1> weightsVec;
        weightsVec << W[0], W[1], W[2], W[3], W[4];
        auto qStart = g_ik->xToQ(startCfg[0], startCfg[1]);
        auto qGoal  = g_ik->xToQ(goalCfg[0],  goalCfg[1]);
        double estimateResol = (qStart - qGoal).cwiseProduct(weightsVec).norm()
                               * g_cfg.motion_validation.cost_resolution_factor;
        motionValidator->setCostResolution(estimateResol);
        si->setMotionValidator(motionValidator);
        si->setup();

        pcsfmtPlanner = std::make_shared<og::PCSFMT>(si, stateSi, g_ik);
        pcsfmtPlanner->setNumSamples(g_cfg.planning.sample_num);
        pcsfmtPlanner->setWeights(const_cast<std::vector<double>&>(g_cfg.weights));

        g_reuseSi = si;
        g_reuseStateSi = stateSi;
        g_reusePlanner = pcsfmtPlanner;
        g_samplesInitialized = true;
    }
    else if (g_samplesInitialized && g_plannerType == PLANNER_PCSFMT)
    {
        OMPL_INFORM("Segment %d: Reusing samples and neighborhoods", segIdx);
        si = g_reuseSi;
        stateSi = g_reuseStateSi;
        pcsfmtPlanner = g_reusePlanner;
        pcsfmtPlanner->resetTreeForNewGoal();
    }
    else if (g_plannerType == PLANNER_FMT)
    {
        auto space      = std::make_shared<ob::RealVectorStateSpace>(paramDim);
        auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(stateDim);

        ob::RealVectorBounds bounds(paramDim);
        bounds.setLow(g_cfg.parameter_space.bounds_low);
        bounds.setHigh(g_cfg.parameter_space.bounds_high);
        space->setBounds(bounds);

        si      = std::make_shared<ob::SpaceInformation>(space);
        stateSi = std::make_shared<ob::SpaceInformation>(stateSpace);

        si->setStateValidityChecker(isStateValid);
        stateSi->setStateValidityChecker(isStateValidForQ);
        si->setup();
    }

    ob::ScopedState<> start(si->getStateSpace()), goal(si->getStateSpace());
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = startCfg[0];
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = startCfg[1];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0]  = goalCfg[0];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1]  = goalCfg[1];

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    if (g_plannerType == PLANNER_PCSFMT)
        pdef->setOptimizationObjective(std::make_shared<ob::EstimatePathLengthOptimizationObjective>(si));
    else
        pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));

    ob::PlannerPtr planner;
    if (g_plannerType == PLANNER_PCSFMT)
    {
        planner = pcsfmtPlanner;
    }
    else
    {
        auto p = std::make_shared<og::FMT>(si);
        p->setNumSamples(g_cfg.planning.sample_num);
        planner = p;
    }
    g_numSample = g_cfg.planning.sample_num;

    planner->setProblemDefinition(pdef);
    planner->setup();

    si->printSettings(std::cout);
    pdef->print(std::cout);

    if (g_plannerType == PLANNER_PCSFMT && segIdx == 1)
    {
        OMPL_INFORM("Adding all waypoints to the sample set before first planning");
        for (const auto &waypoint : g_cfg.trajectory.waypoints)
            pcsfmtPlanner->addWaypointToSamples(waypoint[0], waypoint[1]);
    }

    ompl::time::point t0 = ompl::time::now();
    ob::PlannerStatus solved = planner->ob::Planner::solve(g_cfg.planning.planning_timeout);
    g_planningTime += ompl::time::seconds(ompl::time::now() - t0);
    OMPL_INFORM("Segment %d planning time: %.3f seconds", segIdx,
                ompl::time::seconds(ompl::time::now() - t0));

    std::string plannerTag = (g_plannerType == PLANNER_PCSFMT) ? "PCSFMT" : "FMT";

    if (g_plannerType == PLANNER_PCSFMT)
        static_cast<og::PCSFMT*>(planner.get())->getPlannerDataCsv(
            g_cfg.files.output_dir + "/" + plannerTag + "_planner_data_seg" + std::to_string(segIdx) + ".csv");
    else if (g_plannerType == PLANNER_FMT)
        static_cast<og::FMT*>(planner.get())->getPlannerDataCsv(
            g_cfg.files.output_dir + "/" + plannerTag + "_planner_data_seg" + std::to_string(segIdx) + ".csv");

    if (!solved)
    {
        std::cout << "Segment " << segIdx << ": No solution found" << std::endl;
        return nullptr;
    }

    g_path = pdef->getSolutionPath();
    std::cout << "Segment " << segIdx << ": Found solution:" << std::endl;
    g_path->print(std::cout);

    auto segStatePath = std::make_shared<og::PathGeometric>(stateSi);
    ob::State *s = stateSi->allocState();
    for (auto point : g_path->as<og::PathGeometric>()->getStates())
    {
        auto u = point->as<ob::RealVectorStateSpace::StateType>()->values[0];
        auto v = point->as<ob::RealVectorStateSpace::StateType>()->values[1];
        auto q = g_ik->xToQ(u, v);
        auto stateS = s->as<ob::RealVectorStateSpace::StateType>();
        for (int i = 0; i < 5; ++i) stateS->values[i] = q(i);
        segStatePath->append(s);
    }
    stateSi->freeState(s);

    og::PathSimplifier ps(stateSi);
    ps.smoothBSpline(*segStatePath, g_cfg.bspline.steps, g_cfg.bspline.dt);

    return segStatePath;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <config.yaml>\n";
        return 1;
    }

    if (!g_cfg.loadFromFile(argv[1]))
    {
        std::cerr << "Failed to load config: " << argv[1] << "\n";
        return 1;
    }
    g_cfg.printConfig();

    // 解析规划器类型
    const auto &type = g_cfg.planning.type;
    if (type == "PCSFMT")
        g_plannerType = PLANNER_PCSFMT;
    else if (type == "FMT")
        g_plannerType = PLANNER_FMT;
    else if (type == "AtlasRRTstar")
        g_plannerType = PLANNER_ATLASRRTSTAR;
    else
    {
        std::cerr << "Unknown planner type: " << type << "\n";
        return 1;
    }

    if (g_cfg.planning.random_seed != 0)
        ompl::RNG::setSeed(g_cfg.planning.random_seed);

    const auto &waypoints = g_cfg.trajectory.waypoints;
    if (waypoints.size() < 2)
    {
        std::cerr << "waypoints must contain at least 2 points.\n";
        return 1;
    }

    // 初始化全局对象
    g_nurbs  = new sr::Nurbs(g_cfg.files.pcd_file);
    g_ik     = new dp::InvKin(g_nurbs);
    g_client = new mc::MujocoClient(g_cfg.files.model_file.c_str());

    Eigen::Vector3d surfNormal(g_cfg.output.surface_normal[0],
                               g_cfg.output.surface_normal[1],
                               g_cfg.output.surface_normal[2]);
    g_nurbs->fitSurface(surfNormal);

    const std::string &plannerTag = g_cfg.planning.type;
    const std::string &outputDir  = g_cfg.files.output_dir;

    for (int round = 0; round < g_cfg.planning.planning_rounds; ++round)
    {
        int idx = round + 1;
        g_planningTime = 0.0;

        if (round > 0)
        {
            OMPL_INFORM("Round %d: Resetting planner for new round", idx);
            g_samplesInitialized = false;
            g_reusePlanner.reset();
            g_reuseSi.reset();
            g_reuseStateSi.reset();
        }

        std::shared_ptr<og::PathGeometric> fullPath;
        bool anyFailed = false;

        for (size_t i = 0; i + 1 < waypoints.size(); ++i)
        {
            int segIdx = static_cast<int>(i) + 1;
            OMPL_INFORM("Planning segment %d: [%.2f,%.2f] -> [%.2f,%.2f]",
                segIdx,
                waypoints[i][0], waypoints[i][1],
                waypoints[i+1][0], waypoints[i+1][1]);

            auto segGeo = planSegment(waypoints[i], waypoints[i+1], segIdx);
            if (!segGeo || segGeo->getStateCount() == 0)
            {
                std::cerr << "[round " << idx << "] segment " << segIdx
                          << " planning failed, aborting round.\n";
                anyFailed = true;
                break;
            }

            if (!fullPath)
            {
                fullPath = segGeo;
            }
            else
            {
                const auto &segStates = segGeo->getStates();
                for (size_t j = 1; j < segStates.size(); ++j)
                    fullPath->append(segStates[j]);
            }
        }

        if (anyFailed || !fullPath || fullPath->getStateCount() == 0)
            continue;

        g_statePath = fullPath;
        savePathCsv(g_statePath, outputDir + "/state_path_" + plannerTag + ".csv");
        savePlanningData(idx, g_statePath, g_planningTime,
                         outputDir + "/planning_data_" + plannerTag + ".csv");

        if (g_cfg.rendering.enabled)
        {
            auto rate_render = std::chrono::milliseconds(
                static_cast<int>(1000.0 / g_cfg.rendering.render_frequency));
            auto rate_cal = std::chrono::milliseconds(
                static_cast<int>(1000.0 / g_cfg.rendering.calculation_frequency));
            auto nex_render = std::chrono::steady_clock::now();
            auto nex_cal    = std::chrono::steady_clock::now();
            auto states     = g_statePath->as<og::PathGeometric>()->getStates();
            int  count      = 0;

            while (!glfwWindowShouldClose(g_client->getWindow()))
            {
                auto now = std::chrono::steady_clock::now();
                if (now > nex_render)
                {
                    g_client->render();
                    glfwPollEvents();
                    nex_render += rate_render;
                }
                if (now > nex_cal)
                {
                    if (count < static_cast<int>(states.size()))
                    {
                        const auto statePt = states[count]->as<ob::RealVectorStateSpace::StateType>();
                        g_client->setConfig(std::vector<double>(statePt->values, statePt->values + 5));
                        ++count;
                    }
                    nex_cal += rate_cal;
                }
            }
        }
    }

    if (g_cfg.output.export_visualization)
    {
        saveSurfaceGridCsv(outputDir + "/surface_grid.csv");
        saveWaypointsCsv(outputDir + "/waypoints.csv");
        if (g_statePath)
        {
            savePathContactCsv(g_statePath, outputDir + "/path_contact_" + plannerTag + ".csv");

            std::string pyScript = std::string(__FILE__);
            pyScript = pyScript.substr(0, pyScript.rfind('/')) + "/visualize_path.py";
            std::string pyCmd = "python3 " + pyScript + " " + outputDir + " " + plannerTag;
            OMPL_INFORM("Running visualization: %s", pyCmd.c_str());
            std::system(pyCmd.c_str());
        }
        else
        {
            OMPL_WARN("No solution found, skipping visualization.");
        }
    }

    delete g_client;
    delete g_ik;
    delete g_nurbs;
    return 0;
}
