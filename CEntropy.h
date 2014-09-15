/* 
 * File:   CEntropy.h
 * Author: Jingru
 *
 * Created on February 22, 2012, 12:31 PM
 */

#ifndef CENTROPY_H
#define	CENTROPY_H
#include <ompl/base/Planner.h>
#include <ompl/base/ScopedState.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <statistics/GaussianMixtureModel.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathGeometric.h>
#include "CEPath.h"
#include "Polyhedron.h"
#include <time.h>
#include <statistics/GaussianMixtureModel.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

class CEntropy : public ob::Planner {
public:

    CEntropy(const ob::SpaceInformationPtr &si);
    
    virtual ~CEntropy(void) {
        // free any allocated memory
    }

    /** solve problem, return if ptc becomes true or a better solution is found */
    virtual ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc);
    /** solve one iteration of this problem */
    bool solveIteration(const ob::PlannerTerminationCondition &ptc, double tol = 1e-20);

    virtual void clear(void);

    /** optionally, if additional setup/configuration is needed, the setup() method can be implemented */
    virtual void setup(void);

    /** get all the solution paths */
    virtual void getPlannerData(ob::PlannerData &adata) const;

    /** print the solution path to stream out */
	void printSolution(std::ostream &out);

	/** set number of paths to be sampled and number of states along a path */
    void setNPathsNStates(int numTrajs, int numStatesInTraj);
    /** set start and goal states */
    void setStartandGoalStates();
    /** set dimension and bounds of problem */
    void setEnvironment(int nD, double l, double h);
    /** set a default probem for testing */
    void setDefaultProblem(int nD);
    /** set parameters, number of components and dimension for Gaussian Mixture Model */
    void setGMM(int k, int d);
    /** set standard Gaussian Mixture Model */
    void setStandardGMM();
    /** set min gamma for termination condition */
    void setMinGamma(double mingamma);
    /** set tolerance for termination condition */
    void setTolerance(double tolerance);
    /** set state checker */
    void setStateChecker(ob::StateValidityChecker* checker);

    /** initialize planner parameters */
    void initializePara();
    /** initialize paths */
    bool initializePaths(const ob::PlannerTerminationCondition &ptc);
    
    /** check if start and goal are valid */
    bool isStartGoalValid();
    /** check if a state is valid */
    bool isStateValid(CEState &state);
    /** check if a path is valid */
    bool isPathValid(CEPath &traj);
    /** check if a path segment is valid */
    bool isPathSegmentValid(CEState &s, CEState &e);
    /** check if the diagnal elements of covariance matrix are all zeros */
    bool isGaussianToZero();

    /** sample a feasible state */
    void sampleState(CEState &s);
    /** sample a state */
    void sampleRandomState(CEState &s);
    /** compute path cost of a given traj */
    double pathCost(CEPath &traj);
    /** compute elite set and set the path with min cost to solution */
    void getEliteSet();
    /** train Gaussian Mixture model using the elite set */
    bool trainGMM();
    /** compute mean and covariance, equivalent to trainGMM */
    void computeMeanCov();
    /** generate the set of paths from the trained GMM */
    bool generatePaths(const ob::PlannerTerminationCondition &ptc);

    /** get collision checking time */
    double getColCheckTime();
    /** get solution path */
    CEPath getSolutionPath();
    
    /** termination flag */
    enum Termination{
    	NotConverge,
    	ConvergeTolerence,
    	SolutionGoodEnough,
    	GuassianToZero
    };

    /** flag indicates if CE converges and how it converges */
    Termination terminationFlag;
        
    /** lower and higher bound of environment */
    double low;
    double high;
    /** environment dimensions */
    int numD;
    /** start and goal state */
    CEState start;
    CEState goal;
    
    /** number of GMM components */
    int K;
    /** max number of iterations for GMM EM training */
    int maxGMMIter;
    /** the resolution for checking trajectory  */
    double resolution;
    /** tolerance for CE termination  */
    double tol;

    /** time for collision checking  */
    double colcheckTime;
    /** time for initialization */
    double initTime;

    /** constant for computing the elite set */
    double rho;
    /** rare event level Cost(rho*N) = gama  */
    double gamma;
    /** termination when a solution of cost less than minGama is found */
    double minGamma;
    /** Gaussian Mixture Model */
    Statistics::GaussianMixtureModel GMM;
    /** number of states in a trajectory */
    int nStatesPath;
    /** number of trajectories */
    int nPaths;
    /** set of path sampled */
    vector<CEPath> paths;
    /** store the elite set */
    priority_queue<CEPath*, vector<CEPath*>, PathComparator> eliteset;
    /** the solution found after termination */
    CEPath solution;
    
    /** true if initialization is successful */
    bool initSuccess;
    /** true if a better solution if found */
    bool betterSolution;

    ob::StateValidityChecker* stateChecker;
};

#endif	/* CENTROPY_H */

