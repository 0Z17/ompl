#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/fmt/PCSFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/EstimatePathLengthOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "invkin.h"
#include "nurbs.h"
#include <pcl/io/pcd_io.h>
#include <ompl/util/Time.h>
#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dp = dynamic_planning;
namespace sr = surface_reconstructor;
namespace mc = mujoco_client;

enum PlanningType
{
    FMT,
    PCSFMT
};

std::string pcdFile = "/home/wsl/proj/skyvortex_mujoco/assets/NURBS.pcd";
std::string modelFile = "/home/wsl/proj/skyvortex_mujoco/scene.xml";
const auto nurbs = new sr::Nurbs(pcdFile);
auto ik = new dp::InvKin(nurbs);
mc::MujocoClient client(modelFile.c_str());
std::vector<dynamic_planning::Vector5d> collConfig;
ob::PathPtr path;
ob::PathPtr statePath;

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
    return true;
}

bool isStateValidForQ(const ob::State *state)
{
    // alway return true for test
    // @todo: implement a real state validity checking function

    auto q =state->as<ob::RealVectorStateSpace::StateType>()->values;
    auto vec = std::vector<double>(q, q + 5);
    OMPL_INFORM("state: %f, %f, %f, %f, %f", vec[0], vec[1], vec[2], vec[3], vec[4]);
    auto ret = client.isCollision(std::vector<double>(q, q + 5));
    // OMPL_INFORM("cc result: %s", ret ? "true" : "false");
    // if (ret) collConfig.push_back(q);
    return !ret;
    return true;
}

void plan(PlanningType planning_type)
{
    unsigned int seed = 114514;
    ompl::RNG::setSeed(seed);

    /* params for the planner */
    uint paramDimensionsNum = 2;
    uint stateDimensionsNum = 5;
    // std::string pcdFile = "/home/wsl/proj/pcl/test/milk.pcd";

    auto space(std::make_shared<ob::RealVectorStateSpace>(paramDimensionsNum));
    auto stateSpace(std::make_shared<ob::RealVectorStateSpace>(stateDimensionsNum));

    ob::RealVectorBounds bounds(paramDimensionsNum);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
    auto stateSi(std::make_shared<ob::SpaceInformation>(stateSpace));

    // set the state validity checking for the param space
    si->setStateValidityChecker(isStateValid);
    si->setup();

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

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    // create a random start state
    ob::ScopedState<> start(space);
    // start.random();
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.2;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.2;

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal.random();
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.8;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.8;

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // set the optimization objective
    pdef->setOptimizationObjective(opt);

    // load the inverse kinematics solver
    // const auto nurbs = new sr::Nurbs(pcdFile);
    nurbs->fitSurface();
    // auto ik = new dp::InvKin(nurbs);

    // create a planner for the defined space
    ob::PlannerPtr planner;
    if (planning_type == PCSFMT)
    {
        planner = std::make_shared<og::PCSFMT>(si, stateSi,ik);
        dynamic_cast<og::PCSFMT*>(planner.get())->setNumSamples(5000);
    }
    else if (planning_type == FMT)
    {
        planner = std::make_shared<og::FMT>(si);
        dynamic_cast<og::FMT*>(planner.get())->setNumSamples(5000);
    }


    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    ompl::time::point start_time = ompl::time::now();
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);
    const double planning_time = ompl::time::seconds(ompl::time::now() - start_time);
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
        path->print(std::cout);
    }
    else
    {
        static_cast<og::PCSFMT*>(planner.get())->getPlannerDataCsv("/home/wsl/proj/my_ompl/demos/MyPlanners/"
                                                                      "test_output/PCSFMT_planner_data.csv");
        std::cout << "No solution found" << std::endl;
    }

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
        stateS->values[3] = q(3);
        stateS->values[4] = q(4);

        OMPL_INFORM("state %f, %f, %f, %f, %f", stateS->values[0], stateS->values[1], stateS->values[2],
            stateS->values[3], stateS->values[4]);
        statePath->as<og::PathGeometric>()->append(s->as<ob::State>());
   }

    og::PathSimplifier ps(stateSi);
    ps.smoothBSpline(*statePath->as<og::PathGeometric>(), 3, 0.05);
}

int main(int argc, char **argv)
{
    PlanningType planning_type = PCSFMT;
    // PlanningType planning_type = FMT;
    plan(planning_type);
    for (auto point : statePath->as<og::PathGeometric>()->getStates())
    {
        auto statePt = point->as<ob::RealVectorStateSpace::StateType>();
        client.setConfig(std::vector<double>(statePt->values, statePt->values + 5));
        client.render();
        glfwPollEvents();
        sleep(0.2);
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