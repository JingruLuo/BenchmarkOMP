/*
 * PlannerEntrance.h
 *
 *  Created on: Jan 15, 2014
 *      Author: iuiml
 */

#ifndef PLANNERENTRANCE_H_
#define PLANNERENTRANCE_H_

#include "PolyhedronStateChecker.h"
#include "EnvironmentSettings.h"
#include "PlannerSettings.h"
#include <ompl/base/ProblemDefinition.h>
#include <stdio.h>

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

/** open/close file io */
void openIOFiles(char* pLengthDir, int runs, FILE* SolutionInfoFile, ofstream *SolutionPathFile, ofstream *PlannerDataFile);
void closeIOFiles(FILE* SolutionInfoFile, ofstream *SolutionPathFile, ofstream *PlannerDataFile);

/** get the maximum time allowed for each trial */
double getCutoffSeconds(int nD);

void testLinkage();

/** planner set up */
double MRRTPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, bool do_shortcut);

void RRTStarPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef);

double CEPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, ob::StateValidityChecker* myChecker);

void PRMStarPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, bool lazy);

void FMMPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, ob::StateValidityChecker* myChecker, bool do_offset);


#endif /* PLANNERENTRANCE_H_ */
