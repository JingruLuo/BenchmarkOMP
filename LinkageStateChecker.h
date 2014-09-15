/*
 * LinkageStateChecker.h
 *
 *  Created on: Jan 25, 2014
 *      Author: iuiml
 */

#ifndef LINKAGESTATECHECKER_H_
#define LINKAGESTATECHECKER_H_


#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <math/vector.h>
#include "Linkage.h"

#include "utility.h"

namespace ob = ompl::base;

/**
 * State checker for planar linkage robot
 */
class LinkageStateChecker : public ob::StateValidityChecker
{
public:
	LinkageStateChecker(const ob::SpaceInformationPtr &si):ob::StateValidityChecker(si){
        this->nD = si->getStateDimension();
        linkage = new Linkage(nD);
    }

	~LinkageStateChecker(){
		if(linkage)
			delete linkage;
	}

	/** set link length of planar linkage  */
	void setLinkageLength(vector<double> length){
		linkage->setLinkLength(length);
	}

    /** check if a state is collision free */
    virtual bool isValid(const ob::State *state) const{
        const ob::RealVectorStateSpace::StateType *relstate = state->as<ob::RealVectorStateSpace::StateType>();
        Math::Vector v(nD,relstate->values);
        return isValid(v);
    }

    /** check if a state is collision free */
    bool isValid(const Vector &v) const{
    	linkage->updateConfig(v);
    	return !(linkage->selfCollision());
    }

private:
    /** planar linkage */
    Linkage *linkage;
    /** number of joints (space dimension) */
    int nD;
};

#endif /* LINKAGESTATECHECKER_H_ */
