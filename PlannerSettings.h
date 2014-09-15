/*
 * PlannerSettings.h
 *
 *  Created on: Jan 20, 2014
 *      Author: iuiml
 */

#ifndef PLANNERSETTINGS_H_
#define PLANNERSETTINGS_H_

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include "EnvironmentSettings.h"

namespace ob = ompl::base;

/** perturbe FMM grid by a random number from (FMMOffsetRangeMin,FMMOffsetRangeMax) */
static const double FMMOffsetRangeMax = 0.05;
static const double FMMOffsetRangeMin = 0.01;

//default link length for linkage scenario
static const double default_link_length = 1;

#endif /* PLANNERSETTINGS_H_ */

