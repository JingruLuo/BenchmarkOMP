/* 
 * File:   CEntropy.cpp
 * Author: Jingru
 * 
 * Created on February 22, 2012, 12:31 PM
 */

#include "CEntropy.h"
#include "CEState.h"
#include "CEPath.h"
#include <math/vector.h>
#include <cstdlib>
#include "utility.h"
#include <limits>
#include <time.h>
#include <math/vector.h>
#include<math.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include "PolyhedronStateChecker.h"
#include "PlannerSettings.h"
#include <Timer.h>
#include <algorithm>
#include <limits>

using namespace std;

CEntropy::CEntropy(const ob::SpaceInformationPtr& si) : ob::Planner(si, "CrossEntropy")
{
  // the specifications of this planner (ompl::ob::PlannerSpecs)
  specs_.approximateSolutions = false;
  specs_.optimizingPaths = true;

  this->initializePara();
}

void CEntropy::initializePara()
{
    K = 1;
    numD = ob::Planner::si_->getStateDimension();
//    for scenario NarrowPassage_2_homotopy
//    nStatesPath = 3;
//    rho = 0.05;
//    nPaths = (int)(2*numD*nStatesPath*K/rho);
    //for scenario NarrowPassage_1_homotopy
//    nStatesPath = 3;
//    rho = 0.5;
//    nPaths = 10;//(numD*nStatesPath*K/rho);
    //for NarrowKink_1_homotopy
    nStatesPath = 6;
    rho = 0.5;
    nPaths = 10;

    gamma = 0;
    ob::RealVectorBounds bounds = ob::Planner::si_->getStateSpace()->as<ob::RealVectorStateSpace>()->getBounds();
    low = bounds.low[0];
    high = bounds.high[0];
    minGamma = 0;
    maxGMMIter = 100;
//    resolution = 0.001;
    resolution = ob::Planner::si_->getStateValidityCheckingResolution()*ob::Planner::si_->getMaximumExtent();
    terminationFlag = NotConverge;
    tol = 1.0e-20;
    GMM.Resize(K, nStatesPath*numD);
    colcheckTime = 0;
    initTime = 0;

    initSuccess = false;
    stateChecker = NULL;
    betterSolution = false;
}

void CEntropy::setStartandGoalStates()
{
	const ob::State *st = pis_.nextStart();
	ob::RealVectorStateSpace *rvs = si_->getStateSpace()->as< ob::RealVectorStateSpace>();
	rvs->copyToReals(start.q, st);

	ob::State* tmpState;
	ob::Goal *goalStruct = pdef_->getGoal().get();
	ob::GoalSampleableRegion *goal_s = dynamic_cast<ob::GoalSampleableRegion*> (goalStruct);
	if(goal_s)
	{
		ompl::base::GoalState *tmpGoal = goal_s->as<ompl::base::GoalState > ();
		tmpState = tmpGoal->getState();
	}
	rvs->copyToReals(goal.q, tmpState);
}

void CEntropy::setMinGamma(double mingamma){
	minGamma = mingamma;
}

void CEntropy::setTolerance(double tolerance){
	tol = tolerance;
}

void CEntropy::printSolution(std::ostream& out) {
	out<<"CE Solution"<<endl;
	this->start.print(out);
	this->solution.print(out);
	this->goal.print(out);
	out<<endl;
}

void CEntropy::setStateChecker(ob::StateValidityChecker* checker) {
	this->stateChecker = checker;
}

CEPath CEntropy::getSolutionPath()
{
    CEPath cepath(2+this->nStatesPath);
    cepath.states[0] = this->start;
    for(int i = 0; i < this->nStatesPath; i++)
    {
        cepath.states[i+1] = solution.states[i];
    }
    cepath.states[nStatesPath+1] = this->goal;
    return cepath;
}

ob::PlannerStatus CEntropy::solve(const ob::PlannerTerminationCondition& ptc){
  // make sure the planner is configured correctly; this makes sure there
  // are input states and a goal is specified
  ob::Planner::checkValidity();

  while(ptc == false){
	  if(!initSuccess){
		  if(!initializePaths(ptc))
			  return ob::PlannerStatus::TIMEOUT;
		  initSuccess = true;
	  }
	  betterSolution = false;

	  this->solveIteration(ptc);
	  if(this->terminationFlag != NotConverge){
		  break;
	  }

	  //record the solution found in this iteration
	  // compute a valid motion plan and set it in the goal
	  ob::StateSpacePtr sp = si_->getStateSpace();
	  ob::State *state = ( sp->as<ob::RealVectorStateSpace>()->allocState() );
	  ob::State *state2 = ( sp->as<ob::RealVectorStateSpace>()->allocState() );
	  og::PathGeometric *pathGeometric = new og::PathGeometric(si_);
	  for(int j = 0; j < numD; j++){
		  (state->as<ob::RealVectorStateSpace::StateType>())->values[j] = start.q[j];
		  (state2->as<ob::RealVectorStateSpace::StateType>())->values[j] = goal.q[j];
	  }
	  pathGeometric->append(si_->cloneState(state));

	  for(int i = 0; i < solution.states.size(); i++){
		  for(int j = 0; j < numD; j++){
			  (state->as<ob::RealVectorStateSpace::StateType>())->values[j] = solution.states[i].q[j];
		  }
		  pathGeometric->append(si_->cloneState(state));
	  }
	  pathGeometric->append(si_->cloneState(state2));
	  si_->freeState(state);
	  si_->freeState(state2);
	  ob::Goal *goal   = pdef_->getGoal().get();
	  pdef_->addSolutionPath(ob::PathPtr(pathGeometric));

	  if(betterSolution){
	      break;
	  }
  }

 	return ob::PlannerStatus::EXACT_SOLUTION;
//  return true;
}

void CEntropy::setup(){
	ob::Planner::setup();
	setStartandGoalStates();

    if(!isStartGoalValid()){
        cout<<"Start or Goal is not feasible."<<endl;
        return;
    }
    srand(time(NULL));

}

void CEntropy::getPlannerData(ob::PlannerData& data) const{
  // fill data with the states and edges that were created
  // in the exploration data structure
  // perhaps also fill control::PlannerData
    ob::StateSpacePtr sp = si_->getStateSpace();
      ob::State *state = ( sp->as<ob::RealVectorStateSpace>()->allocState() );
    og::PathGeometric *pathGeometric = new og::PathGeometric(si_);
      for(int j = 0; j < numD; j++){
          (state->as<ob::RealVectorStateSpace::StateType>())->values[j] = start.q[j];
      }
      pathGeometric->append(si_->cloneState(state));
      for(int i = 0; i < solution.states.size(); i++){
          for(int j = 0; j < numD; j++){
              (state->as<ob::RealVectorStateSpace::StateType>())->values[j] = solution.states[i].q[j];
          }     
          pathGeometric->append(si_->cloneState(state));
      }
      for(int j = 0; j < numD; j++){
          (state->as<ob::RealVectorStateSpace::StateType>())->values[j] = goal.q[j];
      }
      pathGeometric->append(si_->cloneState(state));
      si_->freeState(state);
      
    ob::Planner::getPlannerData(data);
    
     for(unsigned int j = 0; j < this->nStatesPath+1; j++){
         data.addEdge(pathGeometric->getState(j),pathGeometric->getState(j+1));
     }
}

void CEntropy::clear(){
  ob::Planner::clear();
}

bool CEntropy::solveIteration(const ob::PlannerTerminationCondition &ptc, double tol){
    if(terminationFlag != NotConverge){
        cout<<"current solution is stored."<<endl;
        return true;
    }         

    double lastCost = numD*numD;
    if(solution.getPathLength() > 0){
    	lastCost = solution.getPathLength();
    }

    //compute elite set
    getEliteSet();

	//check if solution diff < tol for termination
	double curCost = this->pathCost(solution);
	if(abs(lastCost - curCost) < tol){
		cout<<"find a solution with difference less than tolerance "<<tol<<endl;
		terminationFlag = ConvergeTolerence;
		return true;
	}

	//train the probability
//  trainGMM();
	computeMeanCov();

	//check if solution is good enough, i.e., less than a specified cost for termination
	if(gamma <= minGamma){
		cout<<"find a solution with cost less than minGama."<<endl;
		cout<<"gamma="<<gamma<<" <= minGamma="<<minGamma<<endl;
		terminationFlag = SolutionGoodEnough;
		return true;
	}

	//check if diagonal becomes all zeros of covariance matriax for termination
	if(isGaussianToZero()){
		terminationFlag = GuassianToZero;
		cout<<"converged isGaussianToZero"<<endl;
		return true;
	}

	if(generatePaths(ptc) == false)
		return false;

    return true;
}

bool CEntropy::isGaussianToZero(){
	int nzero = 0;
	for(int j=0;j<numD*nStatesPath;j++){
		if(GMM.gaussians[0].L(j,j) <= 1e-3){
		  nzero++;
		  GMM.gaussians[0].L(j,j) = 1e-3;
		}
	}
	if(nzero == numD*nStatesPath){
//            printf("Gaussian dropped to zero\n");
		return true;
	}
	return false;
}
bool CEntropy::isStartGoalValid(){
	if(!isStateValid(start)){
        cout<<"Invalid start."<<endl;
        cout<<"start:"<<endl;start.print();
        return false;
    }
    if(!isStateValid(goal)){
        cout<<"Invalid goal."<<endl;        
        cout<<"goal:"<<endl;goal.print();
        return false;
    }
    return true;
}
    


void CEntropy::setDefaultProblem(int nD){
    K = 1;
    numD = nD;
    rho = 0.05;
    nStatesPath = 3; 
    nPaths = (int)((double)2*numD*nStatesPath*K)/rho;
    gamma = numeric_limits<double>::infinity();
    low = 0;
    high = 1;    
    maxGMMIter = 100;
    resolution = 0.001;
    colcheckTime = 0;
    initTime = 0;
}

void CEntropy::sampleRandomState(CEState& state){
    if(state.q.size() != numD){
        state.q.resize(numD);
    }
    for(int j = 0; j < numD; j++){
        double randnum = low + (high-low)*((double)rand()/(double)RAND_MAX);
        state.q[j] = randnum;
    }
}

void CEntropy::sampleState(CEState& state){
    if(state.q.size() != numD){
        state.q.resize(numD);
    }
    for(int j = 0; j < numD; j++){
        double randnum = low + (high-low)*((double)rand()/(double)RAND_MAX);
        state.q[j] = randnum;
    }
    while(!this->isStateValid(state)){
        for(int j = 0; j < numD; j++){
            double randnum = low + (high-low)*((double)rand()/(double)RAND_MAX);
            state.q[j] = randnum;
        }
    }
}

bool CEntropy::initializePaths(const ob::PlannerTerminationCondition &ptc){
    assert(nPaths > 0);
    paths.clear();
    Timer timer;
    while(ptc == false){
		CEPath path(nStatesPath);
		for(int k = 0; k < nStatesPath; k++){
			CEState state(numD);
			sampleRandomState(state);
			path.states[k] = state;
		}
		if(this->isPathValid(path))
			paths.push_back(path);
		if(paths.size() == nPaths){
			break;
		}
    }
    initTime += timer.ElapsedTime();
    if(paths.size() != nPaths){
    	return false;
    }
    return true;
}

void CEntropy::setEnvironment(int nd, double l, double h){
    numD = nd;
    low = l;
    high = h;
}

void CEntropy::setNPathsNStates(int numTrajs, int numStatesInTraj){
    nPaths = numTrajs;
    nStatesPath = numStatesInTraj;
}

void CEntropy::setGMM(int k, int d){
    GMM.Resize(k, d);
}

void CEntropy::setStandardGMM(){
    Math::Matrix sigma(numD*nStatesPath,numD*nStatesPath);
    sigma.setIdentity();
    for(int i = 0; i < K; i++){
        GMM.gaussians[i].setCovariance(sigma, 1);
    }
}

bool CEntropy::isPathSegmentValid(CEState& s, CEState& e){
    if(normalize(s.q.size(),s.q, e.q) <= resolution)
        return true;
    CEState mid(numD);
    for(int i = 0; i < numD; i++){
        mid.q[i] = (e.q[i]+s.q[i])/2;
    }
    if(!isStateValid(mid))
        return false;
    return (isPathSegmentValid(s,mid) && isPathSegmentValid(mid,e));    
}

bool CEntropy::isPathValid(CEPath& traj){
    double t1 = elapsedTime();
    int size = traj.states.size();
    CEState* last = &start;
    for(int i = 0; i < size; i++){
        if(!isStateValid(traj.states[i])){
            double t2 = elapsedTime();
            this->colcheckTime += t2 - t1;
            return false;
        }
        if(! isPathSegmentValid(*last, traj.states[i])){
            double t2 = elapsedTime();
            this->colcheckTime += t2 - t1;
            return false;
        }
        last = &traj.states[i];
    }
    if( !isPathSegmentValid(*last, goal)){
        double t2 = elapsedTime();
        this->colcheckTime += t2 - t1;
        return false;
    }
    double t2 = elapsedTime();
    this->colcheckTime += t2 - t1;
    return true;
}

bool CEntropy::isStateValid(CEState& state){
    ob::State *omplstate = ( (si_->getStateSpace())->as<ob::RealVectorStateSpace>()->allocState() );
    for(int i = 0; i < numD; i++){
        if(state.q[i] > high || state.q[i] < low)
            return false;
        (omplstate->as<ob::RealVectorStateSpace::StateType>())->values[i] = state.q[i];
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

double CEntropy::getColCheckTime(){
    return colcheckTime;
}

//use the distance of the path as cost temporarily
double CEntropy::pathCost(CEPath& traj){
	return traj.getPathlength(start, goal);
}

void CEntropy::getEliteSet(){
    int eliteSize = min((int)(rho*nPaths) + 2, nPaths-1);
    int bestIndex = -1;
    double bestCost = numeric_limits<double>::max();
    for(int i = 0; i < nPaths; i++){
        double cost = paths[i].getPathlength(start, goal);
        if(bestCost > cost){
        	bestCost = cost;
        	bestIndex = i;
        }
        eliteset.push(&paths[i]);
        if(eliteset.size() > eliteSize){
        	eliteset.pop();
        }
    }

    if(eliteSize > eliteset.size()){
    	cout<<"not enough eliteset"<<endl;
    	abort();
    }

    gamma = eliteset.top()->getPathLength();
    if(solution.getPathLength() > paths[bestIndex].getPathLength()){
    	betterSolution = true;
    }
    solution = paths[bestIndex];
}

bool CEntropy::trainGMM(){
	vector<Math::Vector> examples;
    int eliteSize = eliteset.size();
    while(!eliteset.empty()){
        Math::Vector v;
        eliteset.top()->getPathtoVector(v);
        examples.push_back(v);
        eliteset.pop();
    }

    GMM.Resize(K,numD*nStatesPath);
    Math::Matrix sigma(numD*nStatesPath,numD*nStatesPath);
    sigma.setIdentity();
    for(int i = 0; i < K; i++){
        GMM.gaussians[i].setCovariance(sigma, 1);
    }
    Math::Real tol = 0.1;
    bool res = GMM.TrainEM(examples,tol,10,1);
    return res;
}

void CEntropy::computeMeanCov(){
    int dim = numD*nStatesPath;
    Math::Vector mean(dim);
    mean.setZero();
    int eliteSize = eliteset.size();
    vector<Math::Vector> examples(eliteSize);
    int count = eliteSize-1;
    while(!eliteset.empty()){
        Math::Vector v;
        eliteset.top()->getPathtoVector(v);
//        examples.push_back(v);
        examples[count--] = v;
        mean += v;
       eliteset.pop();
    }

    mean /= eliteSize;
    
    Math::Matrix cov(dim,dim);
    cov.setZero();
    for(int i = 0; i < eliteSize; i++){
        Math::Vector v = examples[i];
        Math::Vector tmp;
        tmp = v - mean;
        Math::Matrix m(dim,dim);
        for(int r = 0; r < dim; r++){
            for(int c = r; c < dim; c++){
                m(r,c) = tmp[r]*tmp[c];
                m(c,r) = m(r,c);
            }
        }
        cov += m;
    }
    cov /= eliteSize;
    
    GMM.Resize(K,numD*nStatesPath);
    assert(GMM.gaussians.size() == 1);
    GMM.gaussians[0].setMean(mean);
    GMM.gaussians[0].setCovariance(cov);
}

//const ob::PlannerTerminationCondition &ptc
bool CEntropy::generatePaths(const ob::PlannerTerminationCondition &ptc){
	Timer timer;
    paths.clear();
    while(ptc == false){
        Math::Vector sample(numD*nStatesPath);
        GMM.Generate(sample);
        CEPath pt;
        pt.setPathfromVector(sample,nStatesPath);
        if(this->isPathValid(pt)){
        	paths.push_back(pt);
        }
        if(paths.size() == nPaths){
        	break;
        }
    }
    if(paths.size() != nPaths){
    	return false;
    }
    return true;
}
