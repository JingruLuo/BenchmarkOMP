/*
 * RRTStarOMPL.cpp
 * implementation is from OMPL, modified for storing extra information
 *
 */

#include "RRTStarOMPL.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/goals/GoalState.h"
#include <limits>

#include "utility.h"

using namespace ompl::geometric;

void RRTStarOMPL::clear(void){
	Planner::clear();
	sampler_.reset();
	freeMemory();
	if (nn_)
		nn_->clear();
}

void RRTStarOMPL::setup(void){
	Planner::setup();

	if (!nn_)
		nn_.reset(new NearestNeighborsGNAT<RRTStarOMPL::Motion*>());
	nn_->setDistanceFunction(boost::bind(&RRTStarOMPL::distanceFunction, this, _1, _2));
}

void RRTStarOMPL::freeMemory(void){
	if (nn_) {
		std::vector<Motion*> motions;
		nn_->list(motions);
		for (unsigned int i = 0; i < motions.size(); ++i) {
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}
}

void RRTStarOMPL::getPlannerData(ob::PlannerData &data) const{
	Planner::getPlannerData(data);
	std::vector<Motion*> motions;
	if (nn_)
		nn_->list(motions);
	for (unsigned int i = 0; i < motions.size(); ++i)
		data.addEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
}

//internal function for finding the size of radius of search neighbor hood.
double RRTStarOMPL::getRadius(void){
	int numDimension = si_->getStateSpace()->getDimension();
	int dataSz = nn_->size();
	double radius = gamma * pow(log((double) (dataSz + 1.0)) / ((double) (dataSz + 1.0)),
		1.0 / (double) (numDimension));
	return radius;
}

/** \brief this is the function for priority Queue used for finding best parent, it will put smaller cost state in front! */
int compareCost(std::pair<og::RRTStarOMPL::Motion*,double> i, std::pair<og::RRTStarOMPL::Motion*,double> j){
	return (i.second < j.second);
}

bool RRTStarOMPL::findBestParent(Motion* rndMotion, std::vector<Motion*> &nbh, Motion*& bestParent)
{
	double t1=elapsedTime();
	//if the nearest neighbor is empty
	if (nbh.size() == 0) {
		//get the nearest one
		bestParent = nn_->nearest(rndMotion);
		//if no nearest neighbor
		if (bestParent == NULL){
			double t2=elapsedTime();
			this->t_nnq+=t2-t1;
			return false;
		}
		//else check the validity between these 2 states
		double time1 = elapsedTime();
		bool resCheckMotion = si_->checkMotion(rndMotion->state, bestParent->state);
		double time2 = elapsedTime();
		t_cc_ += time2 - time1;
		if (!resCheckMotion){
			double t2=elapsedTime();
			this->t_nnq+=t2-t1;
			return false;
		}
	}//or find the best parent among neighbor hood
	else{
		int numNBH = nbh.size();
		//cost and neighbor pair
		std::vector<std::pair<Motion*, double> > stateCostPair(numNBH);
		//an index iterator
		int i = 0;
		//cost of motion
		double trajCost;
		for (std::vector<Motion*>::iterator iter = nbh.begin(); iter != nbh.end(); iter++){
			stateCostPair[i].first = *iter;
			trajCost = distanceFunction((*iter), rndMotion);
			stateCostPair[i].second = (*iter)->costFromRoot + trajCost;
			i++;
		}
		// Sort vertices according to cost
		std::sort(stateCostPair.begin(), stateCostPair.end(), compareCost);
		// Try out each extension according to increasing cost
		i = 0;
		for (std::vector< std::pair<Motion*, double> >::iterator iter = stateCostPair.begin();
			iter != stateCostPair.end(); iter++){

			Motion* motionCurr = iter->first;

			// Extend the current vertex towards stateIn (and this time check for collision with obstacles)
			double time1 = elapsedTime();
			bool resCheckMotion = si_->checkMotion(motionCurr->state, rndMotion->state);
			double time2 = elapsedTime();
			t_cc_ += time2 - time1;
			if (resCheckMotion){
				bestParent = motionCurr;
				double t2 = elapsedTime();
				this->t_nnq += t2 - t1;
				return true;
			}
		}
		
		double t2=elapsedTime();
		this->t_nnq+=t2-t1;
		return false;
	}
	return true;
}

void RRTStarOMPL::checkBetterMotion(Motion* motion, ob::Goal* goal){
	if (goal->isSatisfied(motion->state)){
		double costCurr = motion->costFromRoot;
		if ((lowerBoundMotion == NULL) || ((lowerBoundMotion != NULL) && (costCurr < lowerBoundCost))){
			lowerBoundMotion = motion;
			lowerBoundCost = costCurr;
			foundBetterPath = true;
		}
	}
}

//This is the function for updating children costs during rewiring process!
void RRTStarOMPL::updateBranchCost(Motion* motion, int level, ob::Goal* goal){
	for (std::set< Motion* >::iterator iter = motion->children.begin(); iter != motion->children.end(); iter++){
		Motion* motionCurr = *iter;
		motionCurr->costFromRoot = motion->costFromRoot + motionCurr->costFromParent;
		checkBetterMotion(motionCurr, goal);
		updateBranchCost(motionCurr, level + 1, goal);
	}
}

bool RRTStarOMPL::prepareSolve(){
	checkValidity();

	while (const ob::State * st = pis_.nextStart()){
		Motion *motion = new Motion(si_);
		si_->copyState(motion->state, st);
		nn_->add(motion);
	}

	if (nn_->size() == 0){
		printf("There are no valid initial states!\n");
		return false;
	}

	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	//msg_.inform("Starting with %u states", nn_->size());

	return true;
}

void RRTStarOMPL::sampleAState(ob::State* rndState){
	ob::Goal *goal = pdef_->getGoal().get();
	ob::GoalSampleableRegion *goal_s = dynamic_cast<ob::GoalSampleableRegion*> (goal);
	/* sample random state (with goal biasing) */
	if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
		goal_s->sampleGoal(rndState);
	else
		sampler_->sampleUniform(rndState);
}

bool RRTStarOMPL::shouldInsert(Motion* rndMotion){
	//decide whether or not to insert the state
	if (lowerBoundMotion != NULL){
		double cost2goal = DBL_MAX;
		ob::State* tmpState;
		ob::Goal *goal = pdef_->getGoal().get();
		ob::GoalSampleableRegion *goal_s = dynamic_cast<ob::GoalSampleableRegion*> (goal);
		if (goal_s){
			ob::GoalState *tmpGoal = goal_s->as<ob::GoalState > ();
			tmpState = tmpGoal->getState();
			cost2goal = si_->distance(tmpState, rndMotion->state) - goal_s->getThreshold();
			if (lowerBoundCost < rndMotion->costFromRoot + cost2goal)
				return false;
		}
	}
	return true;
}

void RRTStarOMPL::insertMotion(Motion* bestParent, Motion*& addedMotion, Motion* rndMotion){
	//We have to allocate space for this addedMotion first!
	addedMotion = new Motion(si_);
	si_->copyState(addedMotion->state, rndMotion->state);
	addedMotion->parent = bestParent;
	double costFromParent = 0.0;
	costFromParent = distanceFunction(addedMotion, bestParent);
	addedMotion->costFromParent = costFromParent;
	addedMotion->costFromRoot = bestParent->costFromRoot + costFromParent;
	ob::Goal *goal = pdef_->getGoal().get();
	checkBetterMotion(addedMotion, goal);
	nn_->add(addedMotion);
	//do something about parent
	bestParent->children.insert(addedMotion);
}

void RRTStarOMPL::rewire(std::vector<Motion*>& nbh, Motion* addedMotion){
	double t1 = elapsedTime();
	if (nbh.size() > 0){
		for (std::vector<Motion* >::iterator iter = nbh.begin(); iter != nbh.end(); iter++){
			Motion* motionCurr = *iter;
			double cost = distanceFunction(addedMotion, motionCurr);
			if (addedMotion->costFromRoot + cost < motionCurr->costFromRoot){
				double t1 = elapsedTime();
				bool resCheckMotion = si_->checkMotion(addedMotion->state, motionCurr->state);
				double t2 = elapsedTime();
				t_cc_ += t2 - t1;
				if (resCheckMotion){
					if (motionCurr->parent)
						motionCurr->parent->children.erase(motionCurr);
					motionCurr->parent = addedMotion;
					motionCurr->costFromParent = cost;
					motionCurr->costFromRoot = addedMotion->costFromRoot + cost;
					addedMotion->children.insert(motionCurr);
					ob::Goal *goal = pdef_->getGoal().get();
					updateBranchCost(motionCurr, 0, goal);
					checkBetterMotion(motionCurr, goal);
				} else
					continue;
			}

		}
	}
	double t2 = elapsedTime();
	this->t_rw += t2 - t1;
}

void RRTStarOMPL::storePath(){
	ob::Goal *goal = pdef_->getGoal().get();
	double approxdif = std::numeric_limits<double>::max();
	;
	/* construct the solution path */
	std::vector<Motion*> mpath;
	Motion* tmp = lowerBoundMotion;
	while (tmp != NULL){
		mpath.push_back(tmp);
		tmp = tmp->parent;
	}

	/* set the solution path */
	og::PathGeometric *path = new og::PathGeometric(si_);
	for (int i = mpath.size() - 1; i >= 0; --i){
		if (!mpath[i]->state)
			std::cout << "got you in writing path" << std::endl;
		path->getStates().push_back(si_->cloneState(mpath[i]->state));
	}
	pdef_->addSolutionPath(ob::PathPtr(path));
}

void RRTStarOMPL::interpolate(Motion* bestParent, Motion* rndMotion){
	double d = si_->distance(bestParent->state, rndMotion->state);
	if (d > maxDistance_){
		ob::State *tmpState = si_->allocState();
		si_->getStateSpace()->interpolate(bestParent->state, rndMotion->state, maxDistance_ / d, tmpState);
		//copy tmp state back to rndMotion
		si_->copyState(rndMotion->state, tmpState);
		//free the state
		si_->freeState(tmpState);
	}
}

ob::PlannerStatus RRTStarOMPL::solve(const ob::PlannerTerminationCondition &ptc){
	if (!prepareSolve()) return ob::PlannerStatus::INVALID_START;

	Motion *rndMotion = new Motion(si_);
	ob::State *rndState = rndMotion->state;
	Motion *bestParent = NULL;
	Motion *addedMotion = NULL;
	//nearest neighbor within r
	std::vector<Motion*> nbh;
	double radius;
	while(ptc != false){
		foundBetterPath = false;
		//step 1 sample a random state
		sampleAState(rndState);

		if(si_->isValid(rndState) == false)
			break;

		//step 2 compute near neighbor
		radius = getRadius();
		nn_->nearestR(rndMotion, radius, nbh);

		//step 3 find the best parent
		if (!findBestParent(rndMotion, nbh, bestParent))
			continue;
		//interpolate two motions
		interpolate(bestParent, rndMotion);

//		//decide whether to insert the state
//		if (!shouldInsert(rndMotion))
//			continue;
//		else
		insertMotion(bestParent, addedMotion, rndMotion);

		//step 4 rewiring
		rewire(nbh, addedMotion);

		if(foundBetterPath){
			break;
		}
	}

	if (foundBetterPath && lowerBoundMotion != NULL){
		//std::cout << "we found a path" << std::endl;
		storePath();
	}
		//clean temporary states
	if (rndMotion->state){
		si_->freeState(rndMotion->state);
	}
	delete rndMotion;

//	return pdef_->getGoal().get()->isAchieved();
    if(pdef_->hasSolution())
    	return ob::PlannerStatus::APPROXIMATE_SOLUTION;
    return ob::PlannerStatus::TIMEOUT;

}
