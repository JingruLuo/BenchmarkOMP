/*
 * PRMStar.h
 *
 *  Created on: Sep 7, 2014
 *      Author: jingru
 */

#ifndef PRMSTAR_H_
#define PRMSTAR_H_

#include <ompl/geometric/PathGeometric.h>
#include "planning/OptimalMotionPlanner.h"
#include "PRMStarCSpace.h"
#include <ompl/base/Planner.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * \brief OMPL wrapper for PRMStarPlanner
 */
class PRMStar : public ob::Planner{
public:
	PRMStar(const ob::SpaceInformationPtr &si);
	virtual ~PRMStar(void);

    /** solve problem, return if ptc becomes true or a better solution is found */
    virtual ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc);

    virtual void clear(void);

    /** optionally, if additional setup/configuration is needed, the setup() method can be implemented */
    virtual void setup(void);

    /** get all the solution paths */
    virtual void getPlannerData(ob::PlannerData &adata) const;

    /** set lazy property */
    void setLazy(const bool &ly);

    /** set start and goal configurations */
    void setStartandGoalStates();

    /** get time spent on each step */
    double timeCCheck(){
    	return prmstar->tCheck;
    }
    double timeKNN(){
    	return prmstar->tKnn;
    }
    double timeConnect(){
    	return prmstar->tConnect;
    }
    double timeLazy(){
    	return prmstar->tLazy;
    }
    /** PRMStar configuration space */
    PRMStarCSpace *space;
    /** PRMStarPlanner from KrisLibrary */
    PRMStarPlanner *prmstar;

    /** solution path */
    MilestonePath path;
    /** shortest path length */
    double shortestPathLength;
};

#endif /* PRMSTAR_H_ */
