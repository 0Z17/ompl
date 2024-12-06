#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/fmt/PCSFMT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/EstimatePathLengthOptimizationObjective.h>
#include "invkin.h"
#include "nurbs.h"
#include <pcl/io/pcd_io.h>

#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dp = dynamic_planning;
namespace sr = surface_reconstructor;

bool isStateValid(const ob::State *state)
{
    // alway return true for test
    // @todo: implement a real state validity checking function
    return true;
}

void plan()
{
    /* params for the planner */
    uint paramDimensionsNum = 2;
    uint stateDimensionsNum = 5;
    std::string pcdFile = "/home/wsl/proj/pcl/test/milk.pcd";

    auto space(std::make_shared<ob::RealVectorStateSpace>(paramDimensionsNum));
    auto stateSpace(std::make_shared<ob::RealVectorStateSpace>(stateDimensionsNum));

    ob::RealVectorBounds bounds(paramDimensionsNum);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
    auto stateSi(std::make_shared<ob::SpaceInformation>(space));

    // set the state validity checking for the param space
    si->setStateValidityChecker(isStateValid);

    // set the optimal objective
    ob::OptimizationObjectivePtr opt = std::make_shared<ob::EstimatePathLengthOptimizationObjective>
    (si);

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    // create a random start state
    ob::ScopedState<> start(space);
    start.random();

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal.random();

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // set the optimization objective
    pdef->setOptimizationObjective(opt);

    // load the inverse kinematics solver
    const auto nurbs = new sr::Nurbs(pcdFile);
    nurbs->fitSurface();
    auto ik = new dp::InvKin(nurbs);

    // create a planner for the defined space
    auto planner(std::make_shared<og::PCSFMT>(si, stateSi,ik));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int argc, char **argv)
{
    plan();
    return 0;
}