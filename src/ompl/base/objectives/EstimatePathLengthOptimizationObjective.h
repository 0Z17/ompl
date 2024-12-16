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

/* Author: Luis G. Torres, Jonathan Gammell (allocInformedStateSampler) */

#ifndef OMPL_BASE_OBJECTIVES_ESTIMATE_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_ESTIMATE_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"
#include <Eigen/Dense>
#include "invkin.h"

namespace dp = dynamic_planning;

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective which corresponds to optimizing path length. */
        class EstimatePathLengthOptimizationObjective : public OptimizationObjective
        {
        public:
            EstimatePathLengthOptimizationObjective(const SpaceInformationPtr &si);

            /** \brief Returns identity cost. */
            Cost stateCost(const State *s) const override;

            /** \brief The cost estimated by the tangent-space approximation of the path.
            * @param s1 The first state of the path.
            * @param s2 The second state of the path.
            * @param dqu_s2 The tangent vector along u direction at s2.
            * @param dqv_s2 The tangent vector along v direction at s2.
            * @param weights The weights for the state space dimensions.
            */
            Cost estimateMotionCost(const State *s1, const State *s2,
                dp::Vector5d *dqu_s2, dp::Vector5d *dqv_s2, dp::Vector5d *weights);

            /** \brief Motion cost for this objective is defined as
                the param space distance between \e s1 and \e
                s2, using the method SpaceInformation::distance(). */
            Cost motionCost(const State *s1, const State *s2) const override;

            /** \brief Motion cost for this objective is defined as
                the configuration space distance between \e s1 and \e s2,
                using the method SpaceInformation::distance(). */
            Cost configMotionCost(const State *s1, const State *s2) const;

            /** \brief the motion cost heuristic for this objective is
                simply the configuration space distance between \e s1
                and \e s2, since this is the optimal cost between any
                two states assuming no obstacles. */
            Cost motionCostHeuristic(const State *s1, const State *s2) const override;

            /** \brief the best motion cost estimate for this objective is
                simply the configuration space distance between \e s1
                and \e s2, since this is the optimal cost between any
                two states assuming no obstacles. */
            Cost motionCostBestEstimate(const State *s1, const State *s2) const override;

            /** Set the state space information */
            void setStateSpaceInformation(const SpaceInformationPtr& si);

            /** \brief Print the debug information */
            void printDebugInfo() const;

        protected:
            /** \brief The state space information */
            SpaceInformationPtr stateSi_;

            /** \brief The Debug information */
            unsigned int count_{0};
            double time_{0.0};
        };
    }
}

#endif
