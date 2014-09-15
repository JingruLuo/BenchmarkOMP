/*********************************************************************
 *
 * RRTOMPL.cpp
 * implementation is from OMPL, modified for storing extra information
 *
*********************************************************************/


#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/geometric/PathSimplifier.h"

#include "RRTOMPL.h"
#include "utility.h"
#include <limits>
#include <stdio.h>
#include <iostream>
using namespace std;

using namespace ompl::geometric;

RRTOMPL::RRTOMPL(const ob::SpaceInformationPtr &si) : ob::Planner(si, "RRT"){
    specs_.approximateSolutions = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;

    t_cc_ = 0;
    best_path = NULL;
    iterationCounter_ = 0;
}

void RRTOMPL::clear(void){
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
}

void RRTOMPL::setup(void){
    Planner::setup();
	ompl::tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    if (!nn_)
        nn_.reset(new ompl::NearestNeighborsGNAT<Motion*>());
    nn_->setDistanceFunction(boost::bind(&RRTOMPL::distanceFunction, this, _1, _2));
}

void RRTOMPL::freeMemory(void){
    if (nn_){
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i){
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

ob::PlannerStatus RRTOMPL::solve(const ob::PlannerTerminationCondition &ptc){
    checkValidity();
    ob::Goal                 *goal   = pdef_->getGoal().get();
    ob::GoalSampleableRegion *goal_s = dynamic_cast<ob::GoalSampleableRegion*>(goal);


    while (const ob::State *st = pis_.nextStart()){
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0){
        printf("There are no valid initial states!\n");
        return ob::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();


    Motion *solution  = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion   = new Motion(si_);
    ob::State *rstate = rmotion->state;
    ob::State *xstate = si_->allocState();

    double init_time = elapsedTime();
    double used_time = 0;
    while(ptc == false){
    	iterationCounter_++;
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        ob::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_){
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }
		double t1 = elapsedTime();
        bool resCheckMotion = si_->checkMotion(nmotion->state, dstate);
		double t2 = elapsedTime();
		t_cc_ += t2 - t1;

        if (resCheckMotion){
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;

            nn_->add(motion);
            double dist = 0.0;
            bool solved = goal->isSatisfied(motion->state, &dist);
            if(solved){
                solution = motion;

                /* construct the solution path */
                std::vector<Motion*> mpath;
                while (solution != NULL){
                    mpath.push_back(solution);
                    solution = solution->parent;
                }

                /* set the solution path */
               og::PathGeometric *path = new og::PathGeometric(si_);
			   for (int i = mpath.size() - 1 ; i >= 0 ; --i){
				   path->getStates().push_back(si_->cloneState(mpath[i]->state));
			   }
			   if(pdef_->hasSolution()){
				   double old_length = (pdef_->getSolutionPath())->length();
				   if(path->length() < old_length){
					   pdef_->clearSolutionPaths();
					   pdef_->addSolutionPath(ob::PathPtr(path), true, dist);
					   best_path = path;
				   }else
					   delete path;
			   }else{
				   pdef_->addSolutionPath(ob::PathPtr(path), true, dist);
				   best_path = path;
			   }
                break;
            }
        }
        used_time = elapsedTime() - init_time;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    if(pdef_->hasSolution())
    	return ob::PlannerStatus::APPROXIMATE_SOLUTION;
    return ob::PlannerStatus::TIMEOUT;
}

void RRTOMPL::getPlannerData(ob::PlannerData &data) const{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
        data.addEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
}

