/*
 * PRMStarCSpace.h
 *
 *  Created on: Jan 22, 2014
 *      Author: iuiml
 */

#ifndef MYCSPACE_H_
#define MYCSPACE_H_


#include <math/vector.h>
#include <planning/CSpace.h>
//#include "PolyhedronStateChecker.h"
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
namespace ob = ompl::base;

/**
 * Configuration space for PRMStar planner
 */
class PRMStarCSpace : public CSpace
{
public:
	PRMStarCSpace(const ob::SpaceInformationPtr& si);
	~PRMStarCSpace() {};
	/** set space bounds */
	void SetSpaceBounds(const Config& qmin, const Config& qmax);
	void SetSpaceBounds(const double& qmin, const double& qmax);

	/** set resolution for local planner */
	void setResolution(double r);
	/** sample a random configuration */
	virtual void Sample(Config& x);
	/** check if a configuration is feasible */
	virtual bool IsFeasible(const Config& x);
	/** local planner connecting two configurations */
	virtual EdgePlanner* LocalPlanner(const Config& x,const Config& y);

	/** set state validity checker */
	void setStateValidityChecker(ob::StateValidityChecker* checker);

	/** space information in ompl interface */
	ob::SpaceInformationPtr       si_;
	/** state validity checker */
	ob::StateValidityChecker* stateChecker;
	/** space bounds */
	Config qMin, qMax;
	/** space dimension */
	int nD;
	/** resolution for local planner */
	double resolution;
};

#endif /* MYCSPACE_H_ */

