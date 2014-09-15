/*
 * benchmark.cpp
 *
 *  Created on: Jan 14, 2014
 *      Author: Jingru
 */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/contrib/rrt_star/RRTstar.h>
#include <ompl/base/PlannerTerminationCondition.h>

#include "ompl/datastructures/NearestNeighbors.h"

#include <ompl/config.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <Timer.h>

#include "RRTOMPL.h"
#include "RRTStarOMPL.h"
#include "CEntropy.h"
#include "FMM.h"
#include "PolyhedronStateChecker.h"
#include "EnvironmentSettings.h"
#include "Polyhedron.h"
#include "PlannerEntrance.h"
#include "PlannerSettings.h"
#include "BenchmarkSettings.h"

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * benchmark entrance
 * two summary files are generated:
 * best-path-cost.txt: the best cost given a scenario parameters
 * summary.txt: benchmarking, planner, environment information
 *
 */
int main() {
	srand(time(NULL));
	double t1 = elapsedTime();

	for(int l = 0; l < num_scenarios_to_test; l++){
		ScenarioIndex scenarioIndex = scenarios_to_test[l];
		time_t t = time(0);// get time now
		struct tm * now = localtime( & t );
		//===================create directories=================
		char experimentDir[100];
		sprintf(experimentDir, "%s-%d-%d-%d",scenario_names[(int)scenarioIndex].c_str(),now->tm_year+1900, now->tm_mon+1, now->tm_mday);
		createTopDirs(experimentDir);
		cout<<"Scenario:"<<scenario_names[(int)scenarioIndex]<<endl;
		//===================write experiment info=================
		writeExperimentInfo(experimentDir,scenarioIndex);
		//===================start experiment=================
		for (int i = 0; i < num_nD_to_test; i++) {
			int nD = nD_to_test[i];
			for (int j = 0; j < num_width_to_test; j++) {
				double width = width_to_test[j];
				for (int k = 0; k < num_thin_to_test; k++) {
					double thin = thin_to_test[k];
					//---------------set up problem definition--------------
					//construct space
					ob::StateSpacePtr space(new ob::RealVectorStateSpace(nD));
					// set the bounds for the R^2
					double low, high;
					getScenarioBounds(scenarioIndex,low, high);
					ob::RealVectorBounds bounds(nD);
					bounds.setLow(low);
					bounds.setHigh(high);
					space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
					// construct an instance of  space information from this state space
					ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
					// set state validity checking for this space
					ob::StateValidityChecker* myChecker = setupNarrowPassage(scenarioIndex, si, nD, width, thin);

					// create a start state
					ob::ScopedState<> start(space);
					ob::ScopedState<> goal(space);
					setupStartandGoal(scenarioIndex, nD, thin, start, goal);

					si->setStateValidityChecker(ob::StateValidityCheckerPtr(myChecker));
					si->setStateValidityCheckingResolution(
							stateCheckResolution / si->getStateSpace()->getMaximumExtent());
					si->setup();

					ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
					// set the start and goal states, goal region radius is set to zero
					pdef->setStartAndGoalStates(start, goal);

					//////////////////////////////////////////////////////////////////////
					char pLength[100];
					//---------------Multiple RRT---------------------------
					if(runMRRT){
						sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, MRRTFolder.c_str(), nD, width, thin);
						createDir(pLength);
						double avg = 0;
						for (int i = 1; i <= nRuns; i++) {
							std::cout<<"in multiple-rrt, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
							avg += MRRTPlan(pLength,i, pdef, false);
						}
						cout<<"avg:"<<avg/(double)nRuns<<endl;
					}
					//---------------Multiple RRT with Shortcut---------------------------
					if(runMRRT_shortcut){
						sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, MRRTFolder_shortcut.c_str(), nD, width, thin);
						createDir(pLength);
						double avg = 0;
						for (int i = 1; i <= nRuns; i++) {
							std::cout<<"in multiple-rrt+s, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
							avg += MRRTPlan(pLength,i, pdef, true);
						}
						cout<<"avg:"<<avg/(double)nRuns<<endl;
					}
					//---------------rrt star ---------------------------
					if(runRRTStar){
						sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, RRTStarFolder.c_str(), nD, width, thin);
						createDir(pLength);
						for (int i = 1; i <= nRuns; i++) {
							std::cout<<"in rrt-star, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
							RRTStarPlan(pLength, i, pdef);
						}
					}
					//----------------Cross Entropy-------------------------
					if(runCE){
						sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, CEFolder.c_str(), nD, width, thin);
						createDir(pLength);
						double avg = 0;
						for (int i = 1; i <= nRuns; i++) {
							std::cout<<"in cross entropy, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
							avg += CEPlan(pLength, i, pdef, myChecker);
							cout<<"avg:"<<avg/(double)i<<endl;
						}
						cout<<"avg:"<<avg/(double)nRuns<<endl;
					}
					//----------------PRM star-------------------------
					if(runPRMStar){
						sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, PRMStarFolder.c_str(), nD, width, thin);
						createDir(pLength);
						for (int i = 1; i <= nRuns; i++) {
							std::cout<<"in prm-star, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
							PRMStarPlan(pLength, i, pdef, false);
						}
					}
					//----------------Lazy PRM star-------------------------
					if(runPRMStar_Lazy){
						sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, PRMStarFolder_Lazy.c_str(), nD, width, thin);
						createDir(pLength);
						for (int i = 1; i <= nRuns; i++) {
							std::cout<<"in lazy-prm-star, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
							PRMStarPlan(pLength, i, pdef, true);
						}
					}
					//----------------FMM without offset-------------------------
					if(runFMM){
						sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, FMMFolder.c_str(), nD, width, thin);
						createDir(pLength);
						for (int i = 1; i <= 1; i++) {
							std::cout<<"in fmm, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
							FMMPlan(pLength,i, pdef, myChecker, false);
						}
					}
					//----------------FMM with offset-------------------------
					if(runFMMOffset){
						sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, FMMOffsetFolder.c_str(), nD, width, thin);
						createDir(pLength);
						for (int i = 1; i <= nRuns; i++) {
							std::cout<<"in fmm-offset, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
							bool do_offset = true;
							if(scenarioIndex == PlanaryLinkage)
								do_offset = false;
							FMMPlan(pLength,i, pdef, myChecker, do_offset);
						}
					}
				}
			}
		}
	}
	double t2 = elapsedTime();
	cout<<"Used time:"<<t2-t1<<"(seconds) = "<<(t2-t1)/3600.0<<"(hours)"<<endl;
	cout<<"=====================All Done!====================="<<endl;
	return 0;
}

