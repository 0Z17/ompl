// ============================================================
//  PcsFmtOnly.cpp  —  PCSFMT / FMT 规划器独立测试程序
//  所有可调参数集中在下方 "USER PARAMETERS" 区域
// ============================================================

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/fmt/PCSFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/EstimatePathLengthOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/AdaptiveDiscreteMotionValidator.h>
#include <ompl/util/Time.h>
#include "invkin.h"
#include "nurbs.h"
#include "mujoco_client.h"
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

// ============================================================
//  USER PARAMETERS — 修改此区域内的参数
// ============================================================

// --- 规划算法选择 ---
enum PlannerType { PLANNER_PCSFMT, PLANNER_FMT };
// static const PlannerType PLANNING_TYPE = PLANNER_PCSFMT;
static const PlannerType PLANNING_TYPE = PLANNER_FMT;
// --- 输入/输出路径（也可通过环境变量覆盖）---
static const std::string DEFAULT_PCD_FILE   = "/home/wsl/proj/T_mech_R1/S1/blade_segment.pcd";
static const std::string DEFAULT_MODEL_FILE = "/home/wsl/proj/skyvortex_mujoco/scene.xml";
static const std::string DEFAULT_OUTPUT_DIR = "/home/wsl/proj/T_mech_R1/S1";

// --- 起点 / 终点（参数空间 u,v ∈ [0,1]）---
static const std::vector<double> START_CONFIG = {0.2, 0.2};
static const std::vector<double> GOAL_CONFIG  = {0.8, 0.8};

// --- 代价权重 [X, Y, Z, Psi, Theta] ---
static const std::vector<double> WEIGHTS = {1.0, 1.0, 1.0, 3.0, 6.0};

// --- 参数空间边缘裁剪 ---
static const std::vector<double> CLIP_BOUND = {0.1, 0.9, 0.1, 0.9};

// --- 机械臂参数 ---
static const double LINK_LENGTH = 0.96;


// --- 规划器参数 ---
static const int    SAMPLE_NUM      = 5000;   // 采样点数量
static const double SOLVE_TIMEOUT   = 20.0;   // 规划超时（秒）
static const double RESOL_FACTOR    = 0.05;   // 运动验证分辨率系数

// --- 路径平滑参数 ---
static const int    BSPLINE_STEPS   = 3;
static const double BSPLINE_DT      = 0.0005;

// --- 是否启用 MuJoCo 可视化回放 ---
static const bool   RENDER_RESULT   = true;
static const double RENDER_FPS      = 100.0;  // 渲染帧率
static const double PLAYBACK_FPS    = 5.0;    // 路径回放帧率

// --- 重复规划次数（用于统计实验）---
static const int    PLANNING_ROUNDS = 1;

// ============================================================
//  END OF USER PARAMETERS
// ============================================================

static std::string getEnvVar(const std::string &var, const std::string &defaultValue)
{
    const char *value = std::getenv(var.c_str());
    return value ? std::string(value) : defaultValue;
}

// 全局对象
static std::string pcdFile   = getEnvVar("PCD_FILE",    DEFAULT_PCD_FILE);
static std::string modelFile = getEnvVar("MODEL_FILE",  DEFAULT_MODEL_FILE);
static std::string outputDir = getEnvVar("OUTPUT_DIR",  DEFAULT_OUTPUT_DIR);

static sr::Nurbs        *g_nurbs  = nullptr;
static dp::InvKin       *g_ik     = nullptr;
static mc::MujocoClient *g_client = nullptr;

static ob::PathPtr   g_path;
static ob::PathPtr   g_statePath;
static double        g_planningTime = 0.0;
static unsigned int  g_numSample    = 0u;

// ---- 碰撞检测（参数空间）----
bool isStateValid(const ob::State *state)
{
    auto u = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
    auto v = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
    auto q = g_ik->xToQ(u, v);
    return !g_client->isCollision(std::vector<double>(q.data(), q.data() + q.size()));
}

// ---- 碰撞检测（关节空间）----
bool isStateValidForQ(const ob::State *state)
{
    auto q = state->as<ob::RealVectorStateSpace::StateType>()->values;
    return !g_client->isCollision(std::vector<double>(q, q + 5));
}

// ---- 路径代价计算 ----
double getPathCost(const ob::PathPtr &path)
{
    auto pathStates = path->as<og::PathGeometric>()->getStates();
    double cost = 0.0;
    for (auto it = pathStates.begin(); it != pathStates.end(); ++it)
    {
        if (it == pathStates.begin()) continue;
        const auto cur  = (*it)->as<ob::RealVectorStateSpace::StateType>();
        const auto prev = (*(it-1))->as<ob::RealVectorStateSpace::StateType>();
        double sum = 0.0;
        for (int i = 0; i < 5; ++i)
            sum += std::pow(WEIGHTS[i] * (cur->values[i] - prev->values[i]), 2);
        cost += std::sqrt(sum);
    }
    return cost;
}

// ---- 操作员运动量（Theta 轴）----
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

// ---- 核心规划函数 ----
void plan()
{
    g_ik->setLinkLength(LINK_LENGTH);
    g_ik->setClipBound(CLIP_BOUND);

    const uint paramDim = 2;
    const uint stateDim = 5;

    auto space      = std::make_shared<ob::RealVectorStateSpace>(paramDim);
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(stateDim);

    ob::RealVectorBounds bounds(paramDim);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);

    auto si      = std::make_shared<ob::SpaceInformation>(space);
    auto stateSi = std::make_shared<ob::SpaceInformation>(stateSpace);

    si->setStateValidityChecker(isStateValid);
    stateSi->setStateValidityChecker(isStateValidForQ);

    auto motionValidator = std::make_shared<ob::AdaptiveDiscreteMotionValidator>(si);

    if (PLANNING_TYPE == PLANNER_PCSFMT)
    {
        Eigen::Matrix<double, 5, 1> weightsVec;
        weightsVec << WEIGHTS[0], WEIGHTS[1], WEIGHTS[2], WEIGHTS[3], WEIGHTS[4];
        auto qStart = g_ik->xToQ(START_CONFIG[0], START_CONFIG[1]);
        auto qGoal  = g_ik->xToQ(GOAL_CONFIG[0],  GOAL_CONFIG[1]);
        double estimateResol = (qStart - qGoal).cwiseProduct(weightsVec).norm() * RESOL_FACTOR;
        motionValidator->setCostResolution(estimateResol);
        si->setMotionValidator(motionValidator);
    }

    si->setup();

    g_statePath = std::make_shared<og::PathGeometric>(stateSi);

    // 起点 / 终点
    ob::ScopedState<> start(space), goal(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = START_CONFIG[0];
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = START_CONFIG[1];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0]  = GOAL_CONFIG[0];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1]  = GOAL_CONFIG[1];

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    if (PLANNING_TYPE == PLANNER_PCSFMT)
        pdef->setOptimizationObjective(std::make_shared<ob::EstimatePathLengthOptimizationObjective>(si));
    else
        pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));

    // 创建规划器
    ob::PlannerPtr planner;
    if (PLANNING_TYPE == PLANNER_PCSFMT)
    {
        auto p = std::make_shared<og::PCSFMT>(si, stateSi, g_ik);
        p->setNumSamples(SAMPLE_NUM);
        p->setWeights(const_cast<std::vector<double>&>(WEIGHTS));
        planner = p;
    }
    else
    {
        auto p = std::make_shared<og::FMT>(si);
        p->setNumSamples(SAMPLE_NUM);
        planner = p;
    }
    g_numSample = SAMPLE_NUM;

    planner->setProblemDefinition(pdef);
    planner->setup();

    si->printSettings(std::cout);
    pdef->print(std::cout);

    // 规划
    ompl::time::point t0 = ompl::time::now();
    ob::PlannerStatus solved = planner->ob::Planner::solve(SOLVE_TIMEOUT);
    g_planningTime = ompl::time::seconds(ompl::time::now() - t0);
    OMPL_INFORM("Planning time: %.3f seconds", g_planningTime);

    // 保存规划树数据
    if (PLANNING_TYPE == PLANNER_PCSFMT)
        static_cast<og::PCSFMT*>(planner.get())->getPlannerDataCsv(outputDir + "/PCSFMT_planner_data.csv");
    else
        static_cast<og::FMT*>(planner.get())->getPlannerDataCsv(outputDir + "/FMT_planner_data.csv");

    if (solved)
    {
        g_path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        g_path->print(std::cout);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
        return;
    }

    // 将参数路径转换为关节空间路径
    ob::State *s = stateSi->allocState();
    for (auto point : g_path->as<og::PathGeometric>()->getStates())
    {
        auto u = point->as<ob::RealVectorStateSpace::StateType>()->values[0];
        auto v = point->as<ob::RealVectorStateSpace::StateType>()->values[1];
        auto q = g_ik->xToQ(u, v);
        auto stateS = s->as<ob::RealVectorStateSpace::StateType>();
        for (int i = 0; i < 5; ++i) stateS->values[i] = q(i);
        OMPL_INFORM("state %f %f %f %f %f",
            stateS->values[0], stateS->values[1], stateS->values[2],
            stateS->values[3], stateS->values[4]);
        g_statePath->as<og::PathGeometric>()->append(s);
    }

    // B-Spline 平滑
    og::PathSimplifier ps(stateSi);
    ps.smoothBSpline(*g_statePath->as<og::PathGeometric>(), BSPLINE_STEPS, BSPLINE_DT);
}

int main(int argc, char **argv)
{
    // 初始化全局对象
    g_nurbs  = new sr::Nurbs(pcdFile);
    g_ik     = new dp::InvKin(g_nurbs);
    g_client = new mc::MujocoClient(modelFile.c_str());

    g_nurbs->fitSurface(-Eigen::Vector3d::UnitX());
    // g_nurbs->saveSurfaceAsStl(outputDir + "/surface_EXP.stl");

    for (int round = 0; round < PLANNING_ROUNDS; ++round)
    {
        int idx = round + 1;
        plan();

        if (!g_statePath || g_statePath->as<og::PathGeometric>()->getStateCount() == 0)
        {
            std::cerr << "[round " << idx << "] planning failed, skipping output.\n";
            continue;
        }

        const std::string plannerTag = (PLANNING_TYPE == PLANNER_PCSFMT) ? "PCSFMT" : "FMT";
        savePathCsv(g_statePath, outputDir + "/state_path_" + plannerTag + ".csv");
        savePlanningData(idx, g_statePath, g_planningTime, outputDir + "/planning_data_" + plannerTag + ".csv");

        if (RENDER_RESULT)
        {
            auto rate_render = std::chrono::milliseconds(static_cast<int>(1000.0 / RENDER_FPS));
            auto rate_cal    = std::chrono::milliseconds(static_cast<int>(1000.0 / PLAYBACK_FPS));
            auto nex_render  = std::chrono::steady_clock::now();
            auto nex_cal     = std::chrono::steady_clock::now();
            auto states      = g_statePath->as<og::PathGeometric>()->getStates();
            int  count       = 0;

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

    delete g_client;
    delete g_ik;
    delete g_nurbs;
    return 0;
}
