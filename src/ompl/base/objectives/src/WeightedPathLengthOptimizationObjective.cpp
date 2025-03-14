/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Luis G. Torres, Jonathan Gammell */

#include "ompl/base/objectives/WeightedPathLengthOptimizationObjective.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <memory>

ompl::base::WeightedPathLengthOptimizationObjective::WeightedPathLengthOptimizationObjective
    (const SpaceInformationPtr &si, Eigen::Matrix<double, 5, 1> weights)
  : ompl::base::OptimizationObjective(si)
{
    description_ = "Weighted Path Length";
    weights_ = weights;
    // Setup a default cost-to-go heuristics:
    setCostToGoHeuristic(base::goalRegionCostToGo);
}

ompl::base::Cost ompl::base::WeightedPathLengthOptimizationObjective::stateCost(const State *) const
{
    return identityCost();
}

ompl::base::Cost ompl::base::WeightedPathLengthOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
    const Eigen::Map<Eigen::VectorXd> &config1 = *s1->as<ConstrainedStateSpace::StateType>();
    auto q1 = config1.head(5);
    const Eigen::Map<Eigen::VectorXd> &config2 = *s1->as<ConstrainedStateSpace::StateType>();
    auto q2 = config2.head(5);

    auto cost = Cost((q1 - q2).cwiseProduct(weights_).norm());
    return cost;
}

ompl::base::Cost ompl::base::WeightedPathLengthOptimizationObjective::motionCostHeuristic(const State *s1,
                                                                                  const State *s2) const
{
    return motionCost(s1, s2);
}

ompl::base::Cost ompl::base::WeightedPathLengthOptimizationObjective::motionCostBestEstimate(const State *s1,
                                                                                     const State *s2) const
{
    return motionCost(s1, s2);
}


