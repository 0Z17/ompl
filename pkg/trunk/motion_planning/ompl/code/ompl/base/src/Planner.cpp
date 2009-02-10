/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* \author Ioan Sucan */

#include "ompl/base/Planner.h"
#include "ompl/base/util/random_utils.h"

unsigned int ompl::Planner::getThreadCount(void) const
{
    return m_threadCount;
}

void ompl::Planner::setThreadCount(unsigned int nthreads)
{
    m_threadCount = nthreads > 0 ? nthreads : 1;
    random_utils::setMaxThreads(m_threadCount + 1);
}

bool ompl::Planner::isTrivial(unsigned int *startID, double *distance) const
{
    SpaceInformation::Goal_t goal = m_si->getGoal();
    
    if (!goal)
    {
	m_msg.error("Goal undefined");
	return false;
    }
    
    for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
    {
	SpaceInformation::State_t start = m_si->getStartState(i);
	if (m_si->isValid(start))
	{
	    double dist;
	    if (goal->isSatisfied(start, &dist))
	    {
		if (startID)
		    *startID = i;
		if (distance)
		    *distance = dist;
		return true;
	    }	    
	}
	else
	{
	    m_msg.error("Initial state is in collision!");
	}
    }
    
    return false;    
}

