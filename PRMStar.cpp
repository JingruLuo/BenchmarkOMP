/*
 * PRMStar.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: jingru
 */

#include "PRMStar.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/Planner.h>
#include "utility.h"

PRMStar::PRMStar(const ob::SpaceInformationPtr &si):ob::Planner(si, "PRMStar"){
	si_ = si;
	space = new PRMStarCSpace(si_);
	prmstar = new PRMStarPlanner(space);
	shortestPathLength = numeric_limits<double>::max();
}

PRMStar::~PRMStar(void){
	if(prmstar != NULL){
		delete prmstar;
	}
	if(space != NULL){
		delete space;
	}
}

ob::PlannerStatus PRMStar::solve(const ob::PlannerTerminationCondition &ptc){
	while(ptc == false){
		prmstar->PlanMore();
		MilestonePath newpath;
		bool solvestatus = prmstar->GetPath(newpath);
		if(solvestatus){
			double plength = newpath.Length();
			if(plength < shortestPathLength){
				prmstar->GetPath(path);
				shortestPathLength = plength;
				return ob::PlannerStatus::EXACT_SOLUTION;
			}
		}
	}
	return ob::PlannerStatus::TIMEOUT;
}

void PRMStar::clear(void){

}

void PRMStar::setup(void){
	ob::Planner::setup();
	setStartandGoalStates();
}

void PRMStar::setLazy(const bool &l){
	prmstar->lazy = l;
}

void PRMStar::setStartandGoalStates(){
	vector<double> sConfig, gConfig;

	const ob::State *st = pis_.nextStart();
	ob::RealVectorStateSpace *rvs = si_->getStateSpace()->as< ob::RealVectorStateSpace>();
	rvs->copyToReals(sConfig, st);

	ob::State* tmpState;
	ob::Goal *goalStruct = pdef_->getGoal().get();
	ob::GoalSampleableRegion *goal_s = dynamic_cast<ob::GoalSampleableRegion*> (goalStruct);
	if(goal_s)
	{
		ompl::base::GoalState *tmpGoal = goal_s->as<ompl::base::GoalState > ();
		tmpState = tmpGoal->getState();
	}
	rvs->copyToReals(gConfig, tmpState);
	Config startConfig, goalConfig;
	state_to_config(sConfig.size(), sConfig, startConfig);
	state_to_config(gConfig.size(), gConfig, goalConfig);

	prmstar->Init(startConfig, goalConfig);
}

void PRMStar::getPlannerData(ob::PlannerData& data) const{
  // fill data with the states and edges that were created
  // in the exploration data structure
  // perhaps also fill control::PlannerData
	int nD = si_->getStateDimension();
    ob::StateSpacePtr sp = si_->getStateSpace();
      ob::State *state = ( sp->as<ob::RealVectorStateSpace>()->allocState() );
    og::PathGeometric *pathGeometric = new og::PathGeometric(si_);
      pathGeometric->append(si_->cloneState(state));
      for(int i = 0; i < path.edges.size(); i++){
    	  Config q = path.edges[i]->Start();
          for(int j = 0; j < nD; j++){
              (state->as<ob::RealVectorStateSpace::StateType>())->values[j] = q[i];
          }
          pathGeometric->append(si_->cloneState(state));
      }
	  Config q = path.edges.back()->Goal();
      for(int j = 0; j < nD; j++){
          (state->as<ob::RealVectorStateSpace::StateType>())->values[j] = q[j];
      }
      pathGeometric->append(si_->cloneState(state));
      si_->freeState(state);

    ob::Planner::getPlannerData(data);

     for(unsigned int j = 0; j < path.edges.size() + 1; j++){
         data.addEdge(pathGeometric->getState(j),pathGeometric->getState(j+1));
     }
}
