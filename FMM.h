/* 
 * File:   FMM.h
 * Author: jingru
 *
 * Created on April 1, 2012, 10:08 PM
 */

#ifndef FMM_H
#define	FMM_H
#include <ompl/base/Planner.h>
#include <ompl/base/ScopedState.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathGeometric.h>
#include <structs/arraynd.h>
#include <structs/FixedSizeHeap.h>
#include <utils/indexing.h>
#include <utils/combination.h>
#include <math/math.h>
#include <math/misc.h>
#include <math/vector.h>
#include <Timer.h>
#include <iostream>
#include "Polyhedron.h"
#include <time.h>
#include <structs/arraynd.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;
using namespace Math;
#define DO_TIMING 1

class FMM : public ob::Planner {
public:

	FMM(const ob::SpaceInformationPtr &si);

	virtual ~FMM(void) {
		// free any allocated memory
	}

    /** solve problem, return if ptc becomes true or a solution with a finer grid is found */
	virtual ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc);
	virtual void clear(void);
	/** optionally, if additional setup/configuration is needed, the setup() method can be implemented*/
	virtual void setup(void);
	virtual void getPlannerData(ob::PlannerData &adata) const;

	/**expand the distance from start to all other grids*/
	bool computeDistanceMap(const ob::PlannerTerminationCondition &ptc);

	/** initialize planner parameters*/
	void initializeParameters();
	/* check if a path segment between s and e is valid*/
	bool isPathSegmentValid(const Vector& s, const Vector& e);
	/** check if a path is valid */
	bool isPathValid(const vector< Vector >& path);
	/** check if a state is valid */
	bool isStateValid(const Vector& state);
	/** check if a cell contains collision points from last iteration */
	bool isStateinCollisionCells(const vector<int>& index);

	/** set start and goal state */
	void setStartandGoal();
	/** set state validity checker */
	void setStateChecker(ob::StateValidityChecker* checker);
	/** offset the grid lower bound by parameter offset */
	void setLowerBoundOffset(double offset);

	/** compute the cell index from a state point */
	vector<int> stateToIndex(const Vector& state);
	/** compute the state point given the cell index */
	Vector indexToState(const vector<int>& index);
	/** compute the state point given the cell index */
	Vector indexToState(const Vector& index);

	/** check if start and goal states are valid */
	bool checkStartandGoal();

	/** get the cost of a given state */
	double getStateCost(const Vector& state);
	double getStateCost(const vector<int>& index);
	/** get collision checking time */
	double getColCheckTime();

	/** print the solution path */
	void printSolution(std::ostream &out);

	Real Sum(const Vector& x);

	void printIndex(const vector<int>& index);

	/** Multilinear interpolation of an ND field.
	 * Sensitive to Inf's in the field -- will ignore them
	 */
	Real EvalMultilinear(const ArrayND<Real>& field, const Vector& point);

	/** Gradient descent of an ND field */
	vector<Vector> Descend(const ArrayND<Real>& field, const Vector& start);

	/** compute gradient by finite difference */
	Vector FiniteDifference(const ArrayND<Real>& field,const Vector& x,vector<int>& infDirs);

	/**
	 for three adjacent points at (1,0,0), (0,1,0), (0,0,1) with distances
	 d1, d2, and d3 from the source, finds the minimum distance point to
	 the origin on the diagonal
	 min d(u,v,w) = u*d1 + v*d2 + w*d3 + ||u,v,w||
	 s.t. u+v+w=1
	 */
	Real best_diag_distanceN(const Vector& d);

	/** returns the offsets of all adjacent nodes in the grid */
	void Adjacent(const vector<int>& index, int offset, const ArrayND<Real>& grid, vector<int>& noffsets);

	/* map start and goal to a nearest grid point, set its index as startIndex and goalIndex */
	void mapStatetoNearestGridPoint(const Vector& state, vector<int>& gridIndex);

	/** enumerate the simplices from a grid point */
	struct SimplexEnumerator {
		vector<int> node;
		int nodeOffset;
		const ArrayND<int>& grid;
		//valid axis offsets for each axis (subsets of {-1,1})
		vector<vector<int> > candidates;
		//list of occupied axes
		vector<int> occupied;

		//iteration data
		int level;
		vector<int> axisIndex;
		vector<int> candidateIndex;
		//size of occupied candidates on the current axis
		vector<int> candidateSizes;

		SimplexEnumerator(const vector<int>& _node, const ArrayND<int>& _grid, int gridfilter)
		: grid(_grid) {
			node = _node;
			nodeOffset = grid.indexToOffset(_node);

			//axis aligned candidates
			candidates.resize(node.size());
			vector<int> temp = node;
			for (size_t i = 0; i < node.size(); i++) {
				temp[i] += 1;
				int index = nodeOffset + grid.strides[i];
				if (temp[i] < grid.dims[i] && grid.values[index] == gridfilter)
				candidates[i].push_back(1);
				temp[i] -= 2;
				index = nodeOffset - grid.strides[i];
				if (temp[i] >= 0 && grid.values[index] == gridfilter)
				candidates[i].push_back(-1);
				temp[i] += 1;
			}
			//enumerate combinations of k candidates
			for (size_t i = 0; i < node.size(); i++)
			if (!candidates[i].empty()) occupied.push_back(i);

			reset();
		}

		void reset() {
			level = 1;
			if (occupied.empty()) return;
			axisIndex.resize(1);
			candidateIndex.resize(1);
			axisIndex[0] = 0;
			candidateIndex[0] = 0;
			candidateSizes.resize(1);
			candidateSizes[0] = (int) candidates[occupied[0]].size();
		}

		void get(vector<vector<int> >& res) {
			res.resize(level);
			for (size_t i = 0; i < axisIndex.size(); i++) {
				res[i] = node;
				int axis = axisIndex[i];
				int candidate = candidateIndex[i];
				int origaxis = occupied[axis];
				res[i][origaxis] += candidates[origaxis][candidate];
			}
		}

		void getOffsets(vector<int>& res) {
			res.resize(level);
			for (size_t i = 0; i < axisIndex.size(); i++) {
				int axis = axisIndex[i];
				int candidate = candidateIndex[i];
				int origaxis = occupied[axis];
				res[i] = nodeOffset + candidates[origaxis][candidate] * grid.strides[origaxis];
			}
		}

		bool next() {
			if (level > (int) occupied.size()) return false;
			int nextAxis = IncrementIndex(candidateIndex, candidateSizes);
			if (nextAxis) {
				fill(candidateIndex.begin(), candidateIndex.end(), 0);
				int nextLevel = NextCombination(axisIndex, occupied.size());
				if (nextLevel) {
					level++;
					if (level > (int) occupied.size()) return false;
					axisIndex.resize(level);
					FirstCombination(axisIndex, occupied.size());
					candidateIndex.resize(level);
					fill(candidateIndex.begin(), candidateIndex.end(), 0);
				}
				candidateSizes.resize(level);
				for (size_t i = 0; i < candidateSizes.size(); i++)
				candidateSizes[i] = candidates[occupied[axisIndex[i]]].size();
			}
			return true;
		}
	};

	/** lower and higher bound with offset of environment */
	double lowbound;
	double highbound;
	/** real lower and higher bound of environment */
	double reallowbound;
	double realhighbound;

	/** grid offset for placing an offset grid */
	double gridoffset;
	/** environment dimensions */
	int nD;

	/** number of intervals along each axis */
	int numGrid;
	/** remember the resolution in last iteration */
	int lastnumGrid;
	/** current numGrid = coefficient_nGrid*lastnumGrid */
	double coefficient_nGrid;
	/** resolution of the grid=1/numGrid */
	double grid_resolution;
	/** collision checking resolution */
	double cc_resolution;

	/** distance map */
	ArrayND<double> distances;

	/** round start and goal points to the nearest grid */
	vector<int> startIndex;
	vector<int> goalIndex;
	/** real start and goal states */
	Vector realStart;
	Vector realGoal;

	/** solution path */
	vector<Vector> solution;

	/** if true solution is collision free otherwise in collision */
	bool validflag;

	/** state feasibility checker */
	ob::StateValidityChecker* stateChecker;
	/** time spent on each step */
	double initTime, estimateTime, propagateTime, overheadTime;
	/** store collision checking time */
	double colcheckTime;

	/** store collision points detect from previous iteration */
	vector<Vector> collision_points;
};

#endif	/* FMM_H */

