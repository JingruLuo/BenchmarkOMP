/*
 * PRMStarCSpace.cpp
 *
 *  Created on: Jan 22, 2014
 *      Author: iuiml
 */




#include "PRMStarCSpace.h"
#include <planning/EdgePlanner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
#include <math/random.h>
#include "PlannerSettings.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "utility.h"

PRMStarCSpace::PRMStarCSpace(const ob::SpaceInformationPtr& si) {
	nD = si->getStateDimension();
	qMin.resize(nD);
	qMax.resize(nD);
	qMin.set(-Inf);
	qMax.set(Inf);
	stateChecker = NULL;
	resolution = 0.001;
	si_ = si;

    ob::RealVectorBounds bounds = si_->getStateSpace()->as<ob::RealVectorStateSpace>()->getBounds();
    Config low, high;
    state_to_config(nD, bounds.low, low);
    state_to_config(nD, bounds.high, high);
    SetSpaceBounds(low, high);
    setResolution(si_->getStateValidityCheckingResolution()*si_->getMaximumExtent());
}

void PRMStarCSpace::SetSpaceBounds(const Config& qmin, const Config& qmax) {
	qMin = qmin;
	qMax = qmax;
}

void PRMStarCSpace::setResolution(double r) {
	resolution = r;
}

void PRMStarCSpace::SetSpaceBounds(const double& qmin, const double& qmax) {
	qMin.set(qmin);
	qMax.set(qmax);
}

void PRMStarCSpace::Sample(Config& x) {
	x.resize(nD);
	for(int i = 0; i < nD; i++)
		x[i] = Rand(qMin[i], qMax[i]);
}

void PRMStarCSpace::setStateValidityChecker(ob::StateValidityChecker* checker){
	stateChecker = checker;
}

bool PRMStarCSpace::IsFeasible(const Config& x) {
	ob::State *omplstate = ((si_->getStateSpace())->as<ob::RealVectorStateSpace>()->allocState());
	for (int i = 0; i < nD; i++) {
		(omplstate->as<ob::RealVectorStateSpace::StateType>())->values[i] = x[i];
	}

	bool res;
	if(stateChecker != NULL){
		res = stateChecker->isValid(omplstate);
	}else{
		res = si_->getStateValidityChecker()->isValid(omplstate);
	}
	si_->freeState(omplstate);
	return res;
}

EdgePlanner* PRMStarCSpace::LocalPlanner(const Config& x, const Config& y) {
	return new StraightLineEpsilonPlanner(this, x, y, resolution);
}
