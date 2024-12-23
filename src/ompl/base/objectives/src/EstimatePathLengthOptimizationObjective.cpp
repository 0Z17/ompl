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

#include "ompl/base/objectives/EstimatePathLengthOptimizationObjective.h"
#include <memory>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/Time.h>

ompl::base::EstimatePathLengthOptimizationObjective::EstimatePathLengthOptimizationObjective(const SpaceInformationPtr &si)
  : ompl::base::OptimizationObjective(si)
{
    description_ = "Path Length";

    // Setup a default cost-to-go heuristics:
    setCostToGoHeuristic(base::goalRegionCostToGo);
}

ompl::base::Cost ompl::base::EstimatePathLengthOptimizationObjective::stateCost(const State *) const
{
    return identityCost();
}

ompl::base::Cost ompl::base::EstimatePathLengthOptimizationObjective::estimateMotionCost(const State *s1,
    const State *s2, dp::Vector5d *dqu_s2, dp::Vector5d *dqv_s1, dp::Vector5d* weights)
{
    ompl::time::point start_time = ompl::time::now();
	double du = s2->as<RealVectorStateSpace::StateType>()->values[0]
                - s1->as<RealVectorStateSpace::StateType>()->values[0];
    double dv = s2->as<RealVectorStateSpace::StateType>()->values[1]
                - s1->as<RealVectorStateSpace::StateType>()->values[1];
    dp::Vector5d dq = du * *dqu_s2 + dv * *dqv_s1;
    auto cost = Cost(dq.cwiseProduct(*weights).norm());
    time_ += ompl::time::seconds(ompl::time::now() - start_time);
    count_ += 1;

	return cost;
}

void ompl::base::EstimatePathLengthOptimizationObjective::printDebugInfo() const
{
    // Update the weights used for the motion cost estimate:
    OMPL_INFORM("The time costed by the motion cost estimate is %f seconds.", time_);
    OMPL_INFORM("The number to call the motion cost estimate is %d.", count_);
}

ompl::base::Cost ompl::base::EstimatePathLengthOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
    return Cost(si_->distance(s1, s2));
}

ompl::base::Cost ompl::base::EstimatePathLengthOptimizationObjective::configMotionCost(const State *s1, const State *s2) const
{
    return Cost(stateSi_->distance(s1, s2));
}

ompl::base::Cost ompl::base::EstimatePathLengthOptimizationObjective::motionCostHeuristic(const State *s1,
                                                                                  const State *s2) const
{
    return motionCost(s1, s2);
}

ompl::base::Cost ompl::base::EstimatePathLengthOptimizationObjective::motionCostBestEstimate(const State *s1,
                                                                                     const State *s2) const
{
    return motionCost(s1, s2);
}

void ompl::base::EstimatePathLengthOptimizationObjective::setStateSpaceInformation(const SpaceInformationPtr& si)
{
    stateSi_ = si;
}


