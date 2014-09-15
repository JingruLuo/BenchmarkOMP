/*
 * PlannerEntrance.cpp
 *
 *  Created on: Jan 15, 2014
 *      Author: iuiml
 */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/contrib/rrt_star/RRTstar.h>
#include <ompl/base/PlannerTerminationCondition.h>

#include "ompl/datastructures/NearestNeighbors.h"

#include <math/random.h>
#include <ompl/config.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <Timer.h>

#include "planning/OptimalMotionPlanner.h"
#include "ompl/geometric/PathSimplifier.h"

#include "PlannerEntrance.h"
#include "RRTOMPL.h"
#include "RRTStarOMPL.h"
#include "CEntropy.h"
#include "FMM.h"
#include "EnvironmentSettings.h"
#include "BenchmarkSettings.h"
#include "Polyhedron.h"
#include "utility.h"
#include "PRMStar.h"
#include "Linkage.h"

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

void openIOFiles(char* pLengthDir, int runs, FILE* SolutionInfoFile, ofstream *SolutionPathFile, ofstream *PlannerDataFile){
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile->open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile->open(pLengthName);
	}
}

double getCutoffSeconds(int nD){
	if(nD == 2)
		return cutoff_seconds_2D;
	if(nD == 3)
		return cutoff_seconds_3D;
	return cutoff_seconds_HD;
}

void closeIOFiles(FILE* SolutionInfoFile, ofstream *SolutionPathFile, ofstream *PlannerDataFile){
	fclose(SolutionInfoFile);
	SolutionPathFile->close();
	PlannerDataFile->close();
}

void testLinkage(){
	int nD = 8;
	Config config(nD);
//	double joint_start[] = {150, -90, -50, -80, -90, -100, -120};
	for(int i = 0; i < nD; i++)
		config[i] = joint_start_8D[i]*Math::Pi/180.0;
	vector<double> linklength(1,0);
	linklength.resize(nD);
	for(int i = 0; i < nD; i++)
		linklength[i] = 1;
	Linkage linkage(config, linklength);
	cout << "config:" << config << endl;
	cout<<"joint positions:"<<endl;
	linkage.printJointPos(cout);
	if(linkage.selfCollision()){
		cout<<"In Collision!"<<endl;
	}else
		cout<<"No Collision!"<<endl;
}

double MRRTPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, bool do_shortcut) {
	//============prepare files to output=============
	FILE* SolutionInfoFile;
	ofstream SolutionPathFile;
	ofstream PlannerDataFile;
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile.open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile.open(pLengthName);
	}
	fprintf(SolutionInfoFile, "%%iterations\ttotalTime\tCCTime\tpathLength\tShortcutTime\t\n");
	//=============prepare to run====================
	int nD = pdef->getSpaceInformation()->getStateDimension();
	double cutoff_seconds = getCutoffSeconds(nD);
	//elapse time
	double timeSpentTotal = 0.0;
	double timeSpentCC = 0.0;
	double timeShortcut = 0;
	//best path found
	double bestPathCost = std::numeric_limits<double>::max();

	og::PathGeometric *bestPath = NULL;
	ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(cutoff_seconds);
    og::PathSimplifier pathsimplifier(pdef->getSpaceInformation());
	while(true){
		//===========set up planner========================
		og::RRTOMPL * planner = new og::RRTOMPL(pdef->getSpaceInformation());
		pdef->clearSolutionPaths();
		// set the problem we are trying to solve for the planner
		planner->setProblemDefinition(pdef);
		// perform setup steps for the planner
		planner->setup();
		//======solve=============
		double t1 = elapsedTime();
		ob::PlannerStatus solved = planner->solve(ptc);
		double t2 = elapsedTime();
		timeSpentCC += planner->getCCTime();
		timeSpentTotal += t2 - t1;
		if (solved) {
			ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
			ob::PathPtr path = pdef->getSolutionPath();

		   double pathLength = planner->best_path->length();
		   //do the update
			if (pathLength < bestPathCost){
				if(bestPath != NULL){
					delete bestPath;
				}
				bestPath = new og::PathGeometric(pdef->getSpaceInformation());
				for (int i = 0; i < planner->best_path->getStateCount(); i++){
				   bestPath->getStates().push_back(pdef->getSpaceInformation()->cloneState(planner->best_path->getState(i)));
			    }
			}
		} else {
//			std::cout << "My code shouldn't run here normally" << std::endl;
		}
		if(bestPath != NULL){
		   //short cut the best path
		   if(do_shortcut){
			   double t1 = elapsedTime();
			   pathsimplifier.shortcutPath(*(bestPath));
			   double tmptime = elapsedTime() - t1;
			    timeShortcut += tmptime;
			    timeSpentTotal += tmptime;
		   }
		   double newpathlength = bestPath->length();
		   if(newpathlength < bestPathCost-0.0001){
			   //record result
				bestPathCost = newpathlength;
				fprintf(SolutionInfoFile, "%05d\t%.8e\t%.8e\t%.8e\t%.8e\n",
						planner->getIterationCounter(), timeSpentTotal, timeSpentCC, bestPathCost, timeShortcut);
				fflush(SolutionInfoFile);
				//path cost
				if(recordPath){
					bestPath->print(SolutionPathFile);
					SolutionPathFile.flush();
				}

				if(recordPlannerData){
					ob::PlannerData Data(pdef->getSpaceInformation());
					planner->getPlannerData(Data);
					Data.printGraphviz(PlannerDataFile);
				}
		   }
		}

		if (timeSpentTotal > cutoff_seconds){
			if(!(solved)){
				fprintf(SolutionInfoFile, "%05d\t%.8e\t%.8e\t%.8e\t%.8e\n",
						planner->getIterationCounter(), timeSpentTotal, timeSpentCC, bestPathCost, timeShortcut);
				fflush(SolutionInfoFile);
			}
			break;
		}
		delete planner;
	}

	if(bestPath != NULL){
		delete bestPath;
	}

	fclose(SolutionInfoFile);
	SolutionPathFile.close();
	PlannerDataFile.close();

	return bestPathCost;
}

void RRTStarPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef) {
	//record data tree?
	bool record = false;
	pdef->clearSolutionPaths();
	//=============prepare out put files===========
	FILE* SolutionInfoFile;
	ofstream SolutionPathFile;
	ofstream PlannerDataFile;
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile.open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile.open(pLengthName);
	}
	fprintf(SolutionInfoFile, "%%iterations\t"
			"totalTime\t"
			"CCTime\t"
			"RewireTime\t"
			"NNQtime\t"
			"pathLength\n");
	int nD = pdef->getSpaceInformation()->getStateDimension();
	double cutoff_seconds = getCutoffSeconds(nD);
	//===========set up planner========================
	og::RRTStarOMPL * planner = new og::RRTStarOMPL(pdef->getSpaceInformation());
//	planner->setGama(pdef->getSpaceInformation()->getMaximumExtent()/2.0);
	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);
	// perform setup steps for the planner
	planner->setup();
	//========prepare to run=======================
	//elapse time
	double timeSpentTotal = 0.0;
	double timeSpentCC = 0.0;
	double timeSpentNNQ = 0.0;
	double timeSpentRW = 0.0;

	double bestPathCost = std::numeric_limits<double>::max();
	ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(cutoff_seconds);
	int iter = 0;
	while(true){
		//solve the problem
		double t1 = elapsedTime();
		ob::PlannerStatus solved = planner->solve(ptc);
		//record time
		double t2 = elapsedTime();
		timeSpentTotal += t2 - t1;
		//different from rrtMultiple
		timeSpentCC = planner->getCCTime();
		timeSpentNNQ = planner->getNNQTime();
		timeSpentRW = planner->getRWTime();
		if (solved) {
//            ob::PathPtr path = pdef->getGoal()->getSolutionPath();
			ob::PathPtr path = pdef->getSolutionPath();
			double pathlength = path->length();
			if(pathlength < bestPathCost){
				fprintf(SolutionInfoFile, "%05d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n",
						iter, timeSpentTotal, timeSpentCC, timeSpentRW,
						timeSpentNNQ, pathlength);
				fflush(SolutionInfoFile);
				if(recordPath){
					path->print(SolutionPathFile);
					SolutionPathFile.flush();
				}
				bestPathCost = pathlength;
			}
		}
		if (timeSpentTotal > cutoff_seconds){
			fprintf(SolutionInfoFile, "%05d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n",
					iter, timeSpentTotal, timeSpentCC, timeSpentRW,
					timeSpentNNQ, bestPathCost);
			fflush(SolutionInfoFile);
			break;
		}
		iter++;
	}

	if(recordPlannerData){
		ob::PlannerData Data(pdef->getSpaceInformation());
		planner->getPlannerData(Data);
		Data.printGraphviz(PlannerDataFile);
	}

	delete planner;

	fclose(SolutionInfoFile);
	SolutionPathFile.close();
	PlannerDataFile.close();
}

double CEPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, ob::StateValidityChecker* myChecker) {
	pdef->clearSolutionPaths();
	//=============prepare out put files===========
	FILE* SolutionInfoFile;
	ofstream SolutionPathFile;
	ofstream PlannerDataFile;
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile.open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile.open(pLengthName);
	}
	fprintf(SolutionInfoFile, "%%"
			"totalTime\t"
			"CCTime_planner\t"
			"pathLength\t"
			"InitTime\n");
	int nD = pdef->getSpaceInformation()->getStateDimension();
	double cutoff_seconds = getCutoffSeconds(nD);
	//===========set up planner========================
	CEntropy * planner = new CEntropy(pdef->getSpaceInformation());
	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);
	planner->setStateChecker(myChecker);
	// perform setup steps for the planner
	Timer timer;
	planner->setup();
	double timeSpentTotal = timer.ElapsedTime();
	double bestPathCost = std::numeric_limits<double>::max();
	ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(cutoff_seconds);
	while(true){
		if (planner->terminationFlag != CEntropy::NotConverge){
			break;
		}
		//solve the problem
		timer.Reset();
		ob::PlannerStatus solvestatus = planner->solve(ptc);
		timeSpentTotal += timer.ElapsedTime();

		double timeSpentCC_planner = planner->getColCheckTime();
		if (solvestatus) {
			double pathLength = planner->solution.getPathLength();
			if(pathLength < bestPathCost){
				fprintf(SolutionInfoFile, "%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
						timeSpentCC_planner,
						pathLength, planner->initTime);
				fflush(SolutionInfoFile);
				if(recordPath){
					planner->printSolution(SolutionPathFile);
					SolutionPathFile.flush();
				}
				bestPathCost = pathLength;
			}
		} else {
			fprintf(SolutionInfoFile, "%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
					timeSpentCC_planner,
					bestPathCost, planner->initTime);
			fflush(SolutionInfoFile);

		}
		if (timeSpentTotal > cutoff_seconds){
			if(!(solvestatus)){
				fprintf(SolutionInfoFile, "%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
						timeSpentCC_planner,
						bestPathCost, planner->initTime);
				fflush(SolutionInfoFile);
			}
			break;
		}
	}
	delete planner;

	fclose(SolutionInfoFile);
	SolutionPathFile.close();
	PlannerDataFile.close();
	return bestPathCost;
}


void PRMStarPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, bool lazy) {
	//=============prepare out put files===========
	FILE* SolutionInfoFile;
	ofstream SolutionPathFile;
	ofstream PlannerDataFile;
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile.open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile.open(pLengthName);
	}
	fprintf(SolutionInfoFile, "%%"
			"TotalTime\tCCTime\tKnnTime\tConnectTime\tLazyTime\tpathLength\n");
	//===========set up planner========================
	int nD = pdef->getSpaceInformation()->getStateDimension();
	double cutoff_seconds = getCutoffSeconds(nD);
	const ob::SpaceInformationPtr si_ = pdef->getSpaceInformation();
	PRMStar *planner = new PRMStar(si_);
	planner->setProblemDefinition(pdef);
	planner->setup();
	planner->setLazy(lazy);

	Timer timer;
	double timeSpentTotal = 0;
	ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(cutoff_seconds);
	double bestPathCost = std::numeric_limits<double>::max();
	while(true){
		ob::PlannerStatus solvestatus = planner->solve(ptc);
		timeSpentTotal = timer.ElapsedTime();
		if(solvestatus){
			double pathLength = planner->path.Length();
			if(pathLength < bestPathCost){
				fprintf(SolutionInfoFile, "%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
						planner->timeCCheck(),planner->timeKNN(),planner->timeConnect(),planner->timeLazy(),pathLength);
				fflush(SolutionInfoFile);
				if(recordPath){
					planner->path.Save(SolutionPathFile);
					SolutionPathFile.flush();
				}
				bestPathCost = pathLength;
			}
		} else if(timeSpentTotal > cutoff_seconds){
			fprintf(SolutionInfoFile, "%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
					planner->timeCCheck(),planner->timeKNN(),planner->timeConnect(),planner->timeLazy(),bestPathCost);
			fflush(SolutionInfoFile);
		}
		if (timeSpentTotal > cutoff_seconds)
			break;
	}

	delete planner;

	fclose(SolutionInfoFile);
	SolutionPathFile.close();
	PlannerDataFile.close();
}

void FMMPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, ob::StateValidityChecker* myChecker, bool do_offset) {
	pdef->clearSolutionPaths();
	//===========set up planner========================
	FMM * planner = new FMM(pdef->getSpaceInformation());
	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);
	planner->setStateChecker(myChecker);
	double gridoffset = Math::Rand(FMMOffsetRangeMin,FMMOffsetRangeMax);
	if(do_offset){
		planner->setLowerBoundOffset(-gridoffset);
	}else
		gridoffset = 0;
	// perform setup steps for the planner
	planner->setup();
	//=============prepare out put files===========
	FILE* SolutionInfoFile;
	ofstream SolutionPathFile;
	ofstream PlannerDataFile;
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile.open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile.open(pLengthName);
	}

	fprintf(SolutionInfoFile, "%%"
			"totalTime\t"
			"CCTime_planner\t"
			"numGrid\t"
			"PathValid\t"
			"pathLength\t"
			"offset\t"
			"initTime"
			"estimateTime\t"
			"propagateTime\t"
			"overheadTime\n");
	sprintf(pLengthName,"%s/collisionpoints%04d.txt",pLengthDir,runs);

	int nD = pdef->getSpaceInformation()->getStateDimension();
	double cutoff_seconds = getCutoffSeconds(nD);
	double bestPathCost = std::numeric_limits<double>::max();
	bool bestPathValidFlag = false;
	Vector testq(2);testq[0] = -0.01;testq[1] = 0.5;
	//========prepare to run=======================
	//elapse time
	double timeSpentTotal = 0;
	ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(cutoff_seconds);
	while(true){
		//solve the problem
		Timer timer;
		ob::PlannerStatus solved = planner->solve(ptc);
		//record time
		timeSpentTotal += timer.ElapsedTime();

		double timeSpentCC_planner = planner->getColCheckTime();
		if (solved) {
			double pathLength = getSolutionPathLength(planner->solution);
			fprintf(SolutionInfoFile, "%.8e\t%.8e\t%d\t%d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
					timeSpentCC_planner, planner->numGrid, planner->validflag,
					pathLength,gridoffset,planner->initTime,planner->estimateTime, planner->propagateTime,planner->overheadTime);
			fflush(SolutionInfoFile);
			if(recordPath){
				planner->printSolution(SolutionPathFile);
				SolutionPathFile.flush();
			}
			if(pathLength < bestPathCost){
				bestPathCost = pathLength;
				bestPathValidFlag = planner->validflag;
			}
		} else {
//			double pathLength = getSolutionPathLength(planner->solution);
//						fprintf(SolutionInfoFile, "%.8e\t%.8e\t%d\t%d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
//								timeSpentCC_planner, planner->numGrid, -1,
//								pathLength,gridoffset,planner->initTime,planner->estimateTime, planner->propagateTime,planner->overheadTime);
//						fflush(SolutionInfoFile);
		}
		if (timeSpentTotal > cutoff_seconds){
			if(!(solved)){
				fprintf(SolutionInfoFile, "%.8e\t%.8e\t%d\t%d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
						timeSpentCC_planner, planner->numGrid, bestPathValidFlag,
						bestPathCost,gridoffset,planner->initTime,planner->estimateTime, planner->propagateTime,planner->overheadTime);
				fflush(SolutionInfoFile);
			}
			break;
		}
	}

	delete planner;

	fclose(SolutionInfoFile);
	SolutionPathFile.close();
	PlannerDataFile.close();
}

