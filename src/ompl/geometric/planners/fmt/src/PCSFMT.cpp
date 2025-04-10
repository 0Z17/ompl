/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Autonomous Systems Laboratory, Stanford University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Stanford University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Ashley Clark (Stanford) and Wolfgang Pointner (AIT) */
/* Co-developers: Brice Rebsamen (Stanford), Tim Wheeler (Stanford)
                  Edward Schmerling (Stanford), and Javier V. Gómez (UC3M - Stanford)*/
/* Algorithm design: Lucas Janson (Stanford) and Marco Pavone (Stanford) */
/* Acknowledgements for insightful comments: Oren Salzman (Tel Aviv University),
 *                                           Joseph Starek (Stanford) */

#include <limits>
#include <iostream>

#include <boost/math/constants/constants.hpp>
#include <boost/math/distributions/binomial.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/datastructures/BinaryHeap.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/planners/fmt/PCSFMT.h>
#include <ompl/datastructures/NearestNeighborsLinear.h>

ompl::geometric::PCSFMT::PCSFMT(const base::SpaceInformationPtr &si, const base::SpaceInformationPtr &stateSi, dp::InvKin* invKin)
  : base::Planner(si, "PCSFMT")
{
    // An upper bound on the free space volume is the total space volume; the free fraction is estimated in sampleFree
    freeSpaceVolume_ = si_->getStateSpace()->getMeasure();
    lastGoalMotion_ = nullptr;

    // Set the state space information
    stateSi_ = stateSi;

    // Set the IK solver
    setIK(invKin);

    specs_.approximateSolutions = false;
    specs_.directed = false;

    ompl::base::Planner::declareParam<unsigned int>("num_samples", this, &PCSFMT::setNumSamples, &PCSFMT::getNumSamples,
                                                    "10:10:1000000");
    ompl::base::Planner::declareParam<double>("radius_multiplier", this, &PCSFMT::setRadiusMultiplier,
                                              &PCSFMT::getRadiusMultiplier, "0.1:0.05:50.");
    ompl::base::Planner::declareParam<bool>("use_k_nearest", this, &PCSFMT::setNearestK, &PCSFMT::getNearestK, "0,1");
    ompl::base::Planner::declareParam<bool>("cache_cc", this, &PCSFMT::setCacheCC, &PCSFMT::getCacheCC, "0,1");
    ompl::base::Planner::declareParam<bool>("heuristics", this, &PCSFMT::setHeuristics, &PCSFMT::getHeuristics, "0,1");
    ompl::base::Planner::declareParam<bool>("extended_fmt", this, &PCSFMT::setExtendedPCSFMT, &PCSFMT::getExtendedPCSFMT, "0,1");
}

ompl::geometric::PCSFMT::~PCSFMT()
{
    freeMemory();
}

void ompl::geometric::PCSFMT::setup()
{
    if (pdef_)
    {
        /* Setup the optimization objective. If no optimization objective was
        specified, then default to optimizing path length as computed by the
        distance() function in the state space */
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to estimate optimizing path length.",
                        getName().c_str());
            opt_ = std::make_shared<base::EstimatePathLengthOptimizationObjective>(si_);
            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }

        // objective function class forward
        estOpt_ = dynamic_cast<base::EstimatePathLengthOptimizationObjective*>(opt_.get());

        // Set default weight
        if (weights_ == nullptr)
        {
            const auto weights = new dp::Vector5d;
            *weights << 1.0, 1.0, 1.0, 1.0, 1.0;
            weights_ = weights;
        }

        // The optimization objective needs to be set as the estimated path length optimization objective.
        optForward()->setStateSpaceInformation(stateSi_);
        Open_.getComparisonOperator().opt_ = opt_.get();
        Open_.getComparisonOperator().heuristics_ = heuristics_;

        if (!nn_)
            nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
                // nn_.reset(new NearestNeighborsLinear<Motion *>);
        nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                 {
                                     return distanceFunction(a, b);
                                 });

        if (nearestK_ && !nn_->reportsSortedResults())
        {
            OMPL_WARN("%s: NearestNeighbors datastructure does not return sorted solutions. Nearest K strategy "
                      "disabled.",
                      getName().c_str());
            nearestK_ = false;
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }
}

void ompl::geometric::PCSFMT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        motions.reserve(nn_->size());
        nn_->list(motions);
        for (auto &motion : motions)
        {
            si_->freeState(motion->getState());
            if (motion->hasTangentVec())
            {
                delete motion->getDqu();
                delete motion->getDqv();
            }
            delete motion;
        }
    }

    delete weights_;
}

void ompl::geometric::PCSFMT::clear()
{
    Planner::clear();
    lastGoalMotion_ = nullptr;
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    Open_.clear();
    neighborhoods_.clear();

    // clear the anchor nodes and their weights
    anchorNodesWithWeight_.clear();
    totalAnchorWeight_ = 0.0;
}

void ompl::geometric::PCSFMT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    std::vector<Motion *> motions;
    nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->getState()));

    unsigned int size = motions.size();
    for (unsigned int i = 0; i < size; ++i)
    {
        if (motions[i]->getParent() == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->getState()));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->getParent()->getState()),
                         base::PlannerDataVertex(motions[i]->getState()));
    }
}

void ompl::geometric::PCSFMT::getPlannerDataCsv(const std::string &filename) const
{
    std::vector<Motion *> motions;
    nn_->list(motions);

    std::ofstream file(filename);
    file << "Node,X,Y,Parent,Cost,HeuristicCost\n";

    for (size_t i = 0; i < motions.size(); ++i)
    {
        const Motion *motion = motions[i];
        const base::State *state = motion->getState();
        const double x = state->as<base::RealVectorStateSpace::StateType>()->values[0];
        const double y = state->as<base::RealVectorStateSpace::StateType>()->values[1];
        const double cost = motion->getCost().value();
        const Motion *parentMotion = motion->getParent() ? motion->getParent() : nullptr;
        const double heurCost = motion->getHeuristicCost().value();


        file << motion
        << "," << x
        << "," << y
        << "," << parentMotion
        << "," << cost
        << "," << heurCost
        << "\n";
    }

    file.close();
}

void ompl::geometric::PCSFMT::saveNeighborhood(Motion *m)
{
    if (!m->hasTangentVec())
    {
        setTangentVec(m);
    }
    // Check to see if neighborhood has not been saved yet
    if (neighborhoods_.find(m) == neighborhoods_.end())
    {
        std::vector<Motion *> nbh;
        // if (nearestK_)
        //     nn_->nearestK(m, NNk_, nbh);
        // else
        // @todo: Search with a dynamically adjusted radius
        nn_->nearestR(m, NNr_, nbh);
        if (!nbh.empty())
        {
            // Save the neighborhood but skip the first element, since it will be motion m
            neighborhoods_[m] = std::vector<Motion *>(nbh.size() - 1, nullptr);
            std::copy(nbh.begin() + 1, nbh.end(), neighborhoods_[m].begin());
        }
        else
        {
            // Save an empty neighborhood
            neighborhoods_[m] = std::vector<Motion *>(0);
        }
    }  // If neighborhood hadn't been saved yet
}

// Calculate the unit ball volume for a given dimension
double ompl::geometric::PCSFMT::calculateUnitBallVolume(const unsigned int dimension) const
{
    if (dimension == 0)
        return 1.0;
    if (dimension == 1)
        return 2.0;
    return 2.0 * boost::math::constants::pi<double>() / dimension * calculateUnitBallVolume(dimension - 2);
}

void ompl::geometric::PCSFMT::setTangentVec(Motion *m) const
{
    const auto ik = getIK();
    const auto u = m->getState()->as<base::RealVectorStateSpace::StateType>()->values[0];
    const auto v = m->getState()->as<base::RealVectorStateSpace::StateType>()->values[1];
    const auto q = ik->xToQ(u, v);
    const double psi = q(3);
    const double theta = q(4);
    const auto axis_psi = Eigen::Vector3d(0, 0, 1);
    const auto axis_theta = Eigen::Vector3d(std::cos(psi + M_PI / 2), std::sin(psi + M_PI / 2), 0);
    const double length = ik->getLinkLength();
    const auto vec = - length * Eigen::Vector3d(std::cos(psi)*std::cos(theta),
                                                         std::sin(psi)*std::cos(theta),
                                                         std::sin(theta));
    auto [dqu, dqv] = ik->getTangentVec(u, v);

    // add the linear velocity derivative from dpsi and dtheta
    const Eigen::Vector3d psi_vel = axis_psi.cross(vec);
    const Eigen::Vector3d theta_vel = axis_theta.cross(vec);
    dqu.head(3) += psi_vel * dqu(3);
    dqv.head(3) += psi_vel * dqv(3);
    dqu.head(3) += theta_vel * dqu(4);
    dqv.head(3) += theta_vel * dqv(4);

    // set the tangent vectors
    const auto qPtr = new dp::Vector5d(q);
    const auto dquPtr = new dp::Vector5d(dqu);
    const auto dqvPtr = new dp::Vector5d(dqv);
    m->setQ(qPtr);
    m->setDqu(dquPtr);
    m->setDqv(dqvPtr);
}

double ompl::geometric::PCSFMT::calculateRadius(const unsigned int dimension, const unsigned int n) const
{
    double a = 1.0 / (double)dimension;
    double unitBallVolume = calculateUnitBallVolume(dimension);

    return radiusMultiplier_ * 2.0 * std::pow(a, a) * std::pow(freeSpaceVolume_ / unitBallVolume, a) *
           std::pow(log((double)n) / (double)n, a);
}

void ompl::geometric::PCSFMT::sampleFree(const base::PlannerTerminationCondition &ptc)
{
    unsigned int nodeCount = 0;
    unsigned int sampleAttempts = 0;
    auto *motion = new Motion(si_);

    // Sample numSamples_ number of nodes from the free configuration space
    while (nodeCount < numSamples_ && !ptc)
    {
        sampler_->sampleUniform(motion->getState());

        bool collision_free = si_->isValid(motion->getState());
        sampleAttempts++;

        if (collision_free)
        {
            nodeCount++;
            nn_->add(motion);
            motion = new Motion(si_);
        }  // If collision free
    }      // While nodeCount < numSamples
    si_->freeState(motion->getState());
    delete motion;

    // 95% confidence limit for an upper bound for the true free space volume
    freeSpaceVolume_ = boost::math::binomial_distribution<>::find_upper_bound_on_p(sampleAttempts, nodeCount, 0.05) *
                       si_->getStateSpace()->getMeasure();
}

void ompl::geometric::PCSFMT::sample(const base::PlannerTerminationCondition &ptc)
{
    unsigned int nodeCount = 0;
    unsigned int sampleAttempts = 0;
    auto *motion = new Motion(si_);

    // search the anchor states
    for (double u = 0.0; u < 1.0; u += 1.0/numAnchorNodes_)
    {
        for (double v = 0.0; v < 1.0; v += 1.0/numAnchorNodes_)
        {
            if (ptc)
                break;

            // set the state
            auto state = motion->getState()->as<base::RealVectorStateSpace::StateType>();
            state->values[0] = u;
            state->values[1] = v;
            setTangentVec(motion); // set the tangent vectors
            sampleAttempts++;

            // add the node to the nearest neighbor data structure
            nodeCount++;
            nn_->add(motion);
            
            // calculate weight for the anchor states
            double weight = motion->getDqu()->norm() * motion->getDqu()->norm();
            anchorNodesWithWeight_.emplace_back(motion, weight);
            totalAnchorWeight_ += weight;
            
            motion = new Motion(si_);
        }
    }

    while (nodeCount < numSamples_ && !ptc)
    {
        // select a random anchor node according to its weight
        ompl::RNG rngAnchor;
        double rngWeight = rngAnchor.uniformReal(0.0, totalAnchorWeight_);
        double sumWeight = 0.0;
        Motion* anchorMotion;
        for (auto &anchorNodeWithWeight : anchorNodesWithWeight_)
        {
            if (sumWeight >= rngWeight)
            {
                anchorMotion = anchorNodeWithWeight.first;
                sumWeight += anchorNodeWithWeight.second;
            }
        }

        // sample in the region around the selected anchor node
        auto state = motion->getState();
        auto *rstate = static_cast<base::RealVectorStateSpace::StateType *>(state);
        double dim = si_->getStateSpace()->getDimension();
        for (unsigned int i = 0; i < dim; ++i)
            rstate->values[i] = rngAnchor.uniformReal(bounds.low[i], bounds.high[i]);


        bool collision_free = si_->isValid(motion->getState());
        sampleAttempts++;

        if (collision_free)
        {
            nodeCount++;
            nn_->add(motion);
            motion = new Motion(si_);
        }  // If collision free
    }

    // free the last unused motion object
    si_->freeState(motion->getState());
    delete motion;

}

void ompl::geometric::PCSFMT::assureGoalIsSampled()
{
    // Ensure that there is at least one node near each goal
    while (const base::State *goalState = pis_.nextGoal())
    {
        auto *gMotion = new Motion(si_);
        si_->copyState(gMotion->getState(), goalState);

        // std::vector<Motion *> nearGoal;
        // nn_->nearestR(gMotion, goal->getThreshold(), nearGoal);

        // If there is no node in the goal region, insert one
        // if (nearGoal.empty())
        // {
        OMPL_DEBUG("Setting goal state");
        if (si_->getStateValidityChecker()->isValid(gMotion->getState()))
        {
            nn_->add(gMotion);
            goalState_ = gMotion->getState();
        }
        else
        {
            throw ompl::Exception("Goal state is not valid");
        }
        // }
        // else  // There is already a sample in the goal region
        // {
        //     goalState_ = nearGoal[0]->getState();
        //     si_->freeState(gMotion->getState());
        //     delete gMotion;
        // }
    }  // For each goal
}

ompl::base::PlannerStatus ompl::geometric::PCSFMT::solve(const base::PlannerTerminationCondition &ptc)
{
    if (lastGoalMotion_ != nullptr)
    {
        OMPL_INFORM("solve() called before clear(); returning previous solution");
        traceSolutionPathThroughTree(lastGoalMotion_);
        OMPL_DEBUG("Final path cost: %f", lastGoalMotion_->getCost().value());
        return base::PlannerStatus(true, false);
    }
    if (!Open_.empty())
    {
        OMPL_INFORM("solve() called before clear(); no previous solution so starting afresh");
        clear();
    }

    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    Motion *initMotion = nullptr;

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Add start states to V (nn_) and Open
    while (const base::State *st = pis_.nextStart())
    {
        initMotion = new Motion(si_);
        si_->copyState(initMotion->getState(), st);
        Open_.insert(initMotion);
        initMotion->setSetType(Motion::SET_OPEN);
        initMotion->setCost(opt_->initialCost(initMotion->getState()));
        nn_->add(initMotion);  // V <-- {x_init}
    }

    if (initMotion == nullptr)
    {
        OMPL_ERROR("Start state undefined");
        return base::PlannerStatus::INVALID_START;
    }

    // Sample N free states in the configuration space
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    sampleFree(ptc);
    assureGoalIsSampled();
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    // Calculate the nearest neighbor search radius
    /// \todo Create a PRM-like connection strategy
    // if (nearestK_)
    // {
    // radiusMultiplier_ = 0.5;
    NNk_ = std::ceil(std::pow(2.0 * radiusMultiplier_, (double)si_->getStateDimension()) *
                     (boost::math::constants::e<double>() / (double)si_->getStateDimension()) *
                     log((double)nn_->size()));
    OMPL_DEBUG("Using nearest-neighbors k of %d", NNk_);
    // }
    // else
    // {
    NNr_ = calculateRadius(si_->getStateDimension(), nn_->size());
    OMPL_DEBUG("Using radius of %f", NNr_);
    // }

    // Execute the planner, and return early if the planner returns a failure
    bool plannerSuccess = false;
    bool successfulExpansion = false;
    Motion *z = initMotion;  // z <-- xinit
    saveNeighborhood(z);

    while (!ptc)
    {
        if ((plannerSuccess = goal->isSatisfied(z->getState())))
            break;

        successfulExpansion = expandTreeFromNode(&z);

        if (!extendedPCSFMT_ && !successfulExpansion)
            break;
        if (extendedPCSFMT_ && !successfulExpansion)
        {
            // Apply RRT*-like connections: sample and connect samples to tree
            std::vector<Motion *> nbh;
            std::vector<base::Cost> costs;
            std::vector<base::Cost> incCosts;
            std::vector<std::size_t> sortedCostIndices;

            // our functor for sorting nearest neighbors
            CostIndexCompare compareFn(costs, *opt_);

            auto *m = new Motion(si_);
            while (!ptc && Open_.empty())
            {
                sampler_->sampleUniform(m->getState());

                if (!si_->isValid(m->getState()))
                    continue;

                if (nearestK_)
                    nn_->nearestK(m, NNk_, nbh);
                else
                    nn_->nearestR(m, NNr_, nbh);

                // Get neighbours in the tree.
                std::vector<Motion *> yNear;
                yNear.reserve(nbh.size());
                for (auto &j : nbh)
                {
                    if (j->getSetType() == Motion::SET_CLOSED)
                    {
                        if (nearestK_)
                        {
                            // Only include neighbors that are mutually k-nearest
                            // Relies on NN datastructure returning k-nearest in sorted order
                            const base::Cost connCost = opt_->motionCost(j->getState(), m->getState());
                            const base::Cost worstCost =
                                opt_->motionCost(neighborhoods_[j].back()->getState(), j->getState());

                            if (opt_->isCostBetterThan(worstCost, connCost))
                                continue;
                            yNear.push_back(j);
                        }
                        else
                            yNear.push_back(j);
                    }
                }

                // Sample again if the new sample does not connect to the tree.
                if (yNear.empty())
                    continue;

                // cache for distance computations
                //
                // Our cost caches only increase in size, so they're only
                // resized if they can't fit the current neighborhood
                if (costs.size() < yNear.size())
                {
                    costs.resize(yNear.size());
                    incCosts.resize(yNear.size());
                    sortedCostIndices.resize(yNear.size());
                }

                // Finding the nearest neighbor to connect to
                // By default, neighborhood states are sorted by cost, and collision checking
                // is performed in increasing order of cost
                //
                // calculate all costs and distances
                for (std::size_t i = 0; i < yNear.size(); ++i)
                {
                    incCosts[i] = opt_->motionCost(yNear[i]->getState(), m->getState());
                    costs[i] = opt_->combineCosts(yNear[i]->getCost(), incCosts[i]);
                }

                // sort the nodes
                //
                // we're using index-value pairs so that we can get at
                // original, unsorted indices
                for (std::size_t i = 0; i < yNear.size(); ++i)
                    sortedCostIndices[i] = i;
                std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + yNear.size(), compareFn);

                // collision check until a valid motion is found
                for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                     i != sortedCostIndices.begin() + yNear.size(); ++i)
                {
                    if (si_->checkMotion(yNear[*i]->getState(), m->getState()))
                    {
                        m->setParent(yNear[*i]);
                        yNear[*i]->getChildren().push_back(m);
                        const base::Cost incCost = opt_->motionCost(yNear[*i]->getState(), m->getState());
                        m->setCost(opt_->combineCosts(yNear[*i]->getCost(), incCost));
                        m->setHeuristicCost(opt_->motionCostHeuristic(m->getState(), goalState_));
                        m->setSetType(Motion::SET_OPEN);

                        nn_->add(m);
                        saveNeighborhood(m);
                        updateNeighborhood(m, nbh);

                        Open_.insert(m);
                        z = m;
                        break;
                    }
                }
            }  // while (!ptc && Open_.empty())
        }      // else if (extendedPCSFMT_ && !successfulExpansion)
    }          // While not at goal

    if (plannerSuccess)
    {
        // Return the path to z, since by definition of planner success, z is in the goal region
        lastGoalMotion_ = z;
        traceSolutionPathThroughTree(lastGoalMotion_);

        OMPL_DEBUG("Final path cost: %f", lastGoalMotion_->getCost().value());

        //DEBUG
        static_cast<ompl::base::EstimatePathLengthOptimizationObjective*>(opt_.get())->printDebugInfo();
        printDebugInfo();

        return base::PlannerStatus(true, false);
    }  // if plannerSuccess

    // Planner terminated without accomplishing goal
    return {false, false};
}

void ompl::geometric::PCSFMT::printDebugInfo() const
{
    OMPL_INFORM("The number to call the distance function: %d", numNearestSearching_);
}

void ompl::geometric::PCSFMT::traceSolutionPathThroughTree(Motion *goalMotion)
{
    std::vector<Motion *> mpath;
    Motion *solution = goalMotion;

    // Construct the solution path
    while (solution != nullptr)
    {
        mpath.push_back(solution);
        solution = solution->getParent();
    }

    // Set the solution path
    auto path(std::make_shared<PathGeometric>(si_));
    int mPathSize = mpath.size();
    for (int i = mPathSize - 1; i >= 0; --i)
        path->append(mpath[i]->getState());
    pdef_->addSolutionPath(path, false, -1.0, getName());
}

bool ompl::geometric::PCSFMT::expandTreeFromNode(Motion **z)
{
    // Find all nodes that are near z, and also in set Unvisited

    std::vector<Motion *> xNear;
    const std::vector<Motion *> &zNeighborhood = neighborhoods_[*z];
    const unsigned int zNeighborhoodSize = zNeighborhood.size();
    xNear.reserve(zNeighborhoodSize);

    for (unsigned int i = 0; i < zNeighborhoodSize; ++i)
    {
        Motion *x = zNeighborhood[i];
        if (x->getSetType() == Motion::SET_UNVISITED)
        {
            saveNeighborhood(x);
            // if (nearestK_)
            // {
            // Only include neighbors that are mutually k-nearest
            // Relies on NN datastructure returning k-nearest in sorted order
            const base::Cost connCost = estOpt_->motionCost(x->getState(), (*z)->getState());
            const base::Cost worstCost = opt_->motionCost(neighborhoods_[x].back()->getState(), x->getState());

            if (opt_->isCostBetterThan(worstCost, connCost))
                continue;
            xNear.push_back(x);
            // }
            // else
            //     xNear.push_back(x);
        }
    }

    // For each node near z and in set Unvisited, attempt to connect it to set Open
    std::vector<Motion *> yNear;
    std::vector<Motion *> Open_new;
    const unsigned int xNearSize = xNear.size();
    for (unsigned int i = 0; i < xNearSize; ++i)
    {
        Motion *x = xNear[i];

        // Find all nodes that are near x and in set Open
        const std::vector<Motion *> &xNeighborhood = neighborhoods_[x];

        const unsigned int xNeighborhoodSize = xNeighborhood.size();
        yNear.reserve(xNeighborhoodSize);
        for (unsigned int j = 0; j < xNeighborhoodSize; ++j)
        {
            if (xNeighborhood[j]->getSetType() == Motion::SET_OPEN)
                yNear.push_back(xNeighborhood[j]);
        }

        // Find the lowest cost-to-come connection from Open to x
        base::Cost cMin(opt_->infiniteCost());
        Motion *yMin = getBestParent(x, yNear, cMin);
        yNear.clear();

        // If an optimal connection from Open to x was found
        if (yMin != nullptr)
        {
            bool collision_free = false;
            if (cacheCC_)
            {
                if (!yMin->alreadyCC(x))
                {
                    collision_free = si_->checkMotion(yMin->getState(), x->getState());
                    ++collisionChecks_;
                    // Due to PCSFMT* design, it is only necessary to save unsuccesful
                    // connection attemps because of collision
                    if (!collision_free)
                        yMin->addCC(x);
                }
            }
            else
            {
                ++collisionChecks_;
                collision_free = si_->checkMotion(yMin->getState(), x->getState());
            }

            if (collision_free)
            {
                // Add edge from yMin to x
                x->setParent(yMin);
                x->setCost(cMin);
                x->setHeuristicCost(opt_->motionCostHeuristic(x->getState(), goalState_));
                yMin->getChildren().push_back(x);

                // Add x to Open
                Open_new.push_back(x);
                // Remove x from Unvisited
                x->setSetType(Motion::SET_CLOSED);

                if (debug_x == nullptr && x->getParent()->getParent() != nullptr)
                {
                    debug_x = x;
                }
            }
        }  // An optimal connection from Open to x was found
    }      // For each node near z and in set Unvisited, try to connect it to set Open

    // Update Open
    Open_.pop();
    (*z)->setSetType(Motion::SET_CLOSED);

    // Add the nodes in Open_new to Open
    unsigned int openNewSize = Open_new.size();
    for (unsigned int i = 0; i < openNewSize; ++i)
    {
        Open_.insert(Open_new[i]);
        Open_new[i]->setSetType(Motion::SET_OPEN);
    }
    Open_new.clear();

    if (Open_.empty())
    {
        if (!extendedPCSFMT_)
            OMPL_INFORM("Open is empty before path was found --> no feasible path exists");
        return false;
    }

    // Take the top of Open as the new z
    *z = Open_.top()->data;

    return true;
}

ompl::geometric::PCSFMT::Motion *ompl::geometric::PCSFMT::getBestParent(Motion *m, std::vector<Motion *> &neighbors,
                                                                  base::Cost &cMin)
{
    Motion *min = nullptr;
    const unsigned int neighborsSize = neighbors.size();
    for (unsigned int j = 0; j < neighborsSize; ++j)
    {
        const base::State *s = neighbors[j]->getState();
        // const base::Cost dist = opt_->motionCost(s, m->getState());
        dp::Vector5d* dqu = m->getDqu();
        dp::Vector5d* dqv = m->getDqv();
        const base::Cost dist = estOpt_->estimateMotionCost(s, m->getState(), dqu, dqv, getWeights());
        const base::Cost cNew = opt_->combineCosts(neighbors[j]->getCost(), dist);

        if (opt_->isCostBetterThan(cNew, cMin))
        {
            min = neighbors[j];
            cMin = cNew;
        }
    }
    return min;
}

void ompl::geometric::PCSFMT::updateNeighborhood(Motion *m, const std::vector<Motion *> nbh)
{
    for (auto i : nbh)
    {
        // If CLOSED, the neighborhood already exists. If neighborhood already exists, we have
        // to insert the node in the corresponding place of the neighborhood of the neighbor of m.
        if (i->getSetType() == Motion::SET_CLOSED || neighborhoods_.find(i) != neighborhoods_.end())
        {
            const base::Cost connCost = opt_->motionCost(i->getState(), m->getState());
            const base::Cost worstCost = opt_->motionCost(neighborhoods_[i].back()->getState(), i->getState());

            if (opt_->isCostBetterThan(worstCost, connCost))
                continue;

            // Insert the neighbor in the vector in the correct order
            std::vector<Motion *> &nbhToUpdate = neighborhoods_[i];
            for (std::size_t j = 0; j < nbhToUpdate.size(); ++j)
            {
                // If connection to the new state is better than the current neighbor tested, insert.
                const base::Cost cost = opt_->motionCost(i->getState(), nbhToUpdate[j]->getState());
                if (opt_->isCostBetterThan(connCost, cost))
                {
                    nbhToUpdate.insert(nbhToUpdate.begin() + j, m);
                    break;
                }
            }
        }
        else
        {
            std::vector<Motion *> nbh2;
            if (nearestK_)
                nn_->nearestK(m, NNk_, nbh2);
            else
                nn_->nearestR(m, NNr_, nbh2);

            if (!nbh2.empty())
            {
                // Save the neighborhood but skip the first element, since it will be motion m
                neighborhoods_[i] = std::vector<Motion *>(nbh2.size() - 1, nullptr);
                std::copy(nbh2.begin() + 1, nbh2.end(), neighborhoods_[i].begin());
            }
            else
            {
                // Save an empty neighborhood
                neighborhoods_[i] = std::vector<Motion *>(0);
            }
        }
    }
}

// double ompl::geometric::PCSFMT::distanceFunction(const Motion *a, const Motion *b)
// {
//     numNearestSearching_++;
//     dp::Vector5d* dqu = b->getDqu();
//     dp::Vector5d *dqv = b->getDqv();
//     const auto estOpt = optForward();
//     return estOpt->estimateMotionCost(a->getState(), b->getState(), dqu, dqv, getWeights()).value();
// }

double ompl::geometric::PCSFMT::distanceFunction(const Motion *a, const Motion *b)
{
    numNearestSearching_++;
    return opt_->motionCost(a->getState(), b->getState()).value();
}

