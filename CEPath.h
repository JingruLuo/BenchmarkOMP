/* 
 * File:   ParameterizedTrajectory.h
 * Author: jingru
 *
 * Created on February 19, 2012, 10:27 AM
 */

#ifndef CEPATH_H
#define	CEPATH_H
#include "CEState.h"
#include "utility.h"
#include <vector>
#include <iostream>
#include <math/vector.h>
#include <limits>
using namespace std;

/**
 * Path representation
 */
class CEPath {
public:
	CEPath() {
		pathlength = numeric_limits<double>::max();
	}
	CEPath(int _nStates) {
		states.resize(_nStates);
	}
	CEPath(const CEPath& orig) {
		this->states = orig.states;
		for (int i = 0; i < states.size(); i++) {
			this->states[i] = orig.states[i];
		}
	}
	void print() {
		for (int i = 0; i < states.size(); i++) {
			((CEState) states[i]).print();
		}
	}
	void print(std::ostream &out) {
		for (int i = 0; i < states.size(); i++) {
			((CEState) states[i]).print(out);
		}
	}

	void print(std::ostream &out) const {
		out << "CEPath of " << states.size() << " states" << std::endl;
		for (unsigned int i = 0; i < states.size(); ++i)
			((CEState) states[i]).print();
		out << std::endl;
	}

	~CEPath() {
		freememory();
	}
	void freememory() {
		int nStates = states.size();
		for (int i = 0; i < nStates; i++) {
			states[i].q.clear();
		}
		states.clear();
	}

	double getPathLength(){
		return pathlength;
	}
	double getPathlength(CEState &start, CEState &goal) {
		pathlength = 0;
		CEState *last = &start;
		int nStates = states.size();
		for (int i = 0; i < nStates; i++) {
			pathlength += normalize(start.size(), last->q, states[i].q);
			last = &states[i];
		}
		pathlength += normalize(start.size(), last->q, goal.q);
		return pathlength;
	}

	void getPathtoVector(Math::Vector &v) {
		int nStates = states.size();
		if (states.empty()) {
			cout << "The trajectory is empty." << endl;
			return;
		}
		int numD = states[0].q.size();
		v.resize(nStates * numD);
		for (int i = 0; i < nStates; i++) {
			for (int j = 0; j < numD; j++) {
				v[i * numD + j] = states[i].q[j];
			}
		}
	}

	/** get a trajectory from a vector as (state1, state2, ...) */
	void setPathfromVector(Math::Vector &v, int _nStates) {
		int nStates = _nStates;
		if (!states.empty())
			states.clear();
		states.resize(nStates);
		int numD = v.size() / nStates;
		for (int i = 0; i < nStates; i++) {
			CEState s(numD);
			for (int j = 0; j < numD; j++) {
				s.q[j] = v[i * numD + j];
			}
			states[i] = s;
		}
	}

	/** path of states */
	vector<CEState> states;
	/** path length from start to goal */
	double pathlength;
};

class PathComparator{
public:
	bool operator ()(const CEPath *p1, const CEPath *p2){
		return p1->pathlength > p2->pathlength;
	}
};


#endif	/* PARATRAJECTORY_H */

