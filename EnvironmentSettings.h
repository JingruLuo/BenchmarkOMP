/*
 * File:   EnvironmentSettings.h
 * Author: Jingru
 *
 * Created on March 13, 2012, 11:35 AM
 */

#ifndef READENVIRONMENT_H
#define	READENVIRONMENT_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>

#include <ompl/base/ScopedState.h>
#include <math/vector.h>
#include "Polyhedron.h"
#include "CEState.h"

namespace ob = ompl::base;

/** scenario indexes */
enum ScenarioIndex{
	NarrowPassage_2_homotopy,
	NarrowPassage_1_homotopy,
	PlanaryLinkage,
	NarrowKink_1_homotopy
};

/** scenario names */
static const string scenario_names[] = {"NP_2_homotopy", "NP_1_homotopy", "PlanaryLinkage", "Kink"};

/** default width and length/thickness of narrow passage */
static const double width_global = 0.2;
static const double thin_global = 0.1;

/** default values for the middle of the space and the narrow passage
 * used for setting up nnarrow passage with different width and length/thickness
 */
static const double onefourth_global = 0.25;
static const double onesecond_global = 0.5;

/** joint values of start and goal positions for planary linkage from 5D-8D */
static const double joint_start_5D[] = {120, -90, -80, -110, -120};
static const double joint_goal_5D[] = {179, 0, 0, 0, 0};

static const double joint_start_6D[] = {120, -90, -60, -110, -80, -130};
static const double joint_goal_6D[] = {179, 0, 0, 0, 0, 0};

static const double joint_start_7D[] = {150, -90, -50, -80, -90, -100, -130};
static const double joint_goal_7D[] = {179, 0, 0, 0, 0, 0, 0};

static const double joint_start_8D[] = {150, -90, -50, -80, -80, -80, -100, -125};
static const double joint_goal_8D[] = {179, 0, 0, 0, 0, 0, 0, 0};

/** number of configurations in the best path of benchmark problem kink */
static const int num_config_best_path_kink = 8;

/** resolution of collision checking along a path */
const double stateCheckResolution = 0.001;

/** get the best path length according the bencmark problem,
 * dimension, width, length of narrow passage
 */
double getBestPathCost(ScenarioIndex scenario, int nD, double width, double thin, double valuePerturb = 0);

double getBestPathCost_Linkage(int nD);

double getBestPathCost_Kink(int nD, double width, double thin, double valuePerturb = 0);

double getBestPathCost_2_homotopy(int nD, double width, double thin, double valuePerturb = 0);

double getBestPathCost_1_homotopy(int nD, double width, double thin, double valuePerturb = 0);

void getScenarioBounds(ScenarioIndex scenario, double &low, double &high);

/** set up narrow passage with specified parameters and return a state validity checker */
ob::StateValidityChecker* setupNarrowPassage(ScenarioIndex scenario, ob::SpaceInformationPtr &si, const int &nD, const double &width, const double &thin, double valuePerturb = 0);

/** width is the width of narrow passage,
 * thin is the horizontal length of the horizontal narrow passage
 */
vector<Polyhedron> setupNarrowPassage_Kink(const int &nD, const double &width, double thin, double valuePerturb = 0);

vector<Polyhedron> setupNarrowPassage_2_homotopy(const int &nD, const double &width, const double &thin, double valuePerturb = 0);

vector<Polyhedron> setupNarrowPassage_1_homotopy(const int &nD, const double &width, const double &thin, double valuePerturb = 0);

/** set up start and goal positions for each scenario */
void setupStartandGoal(ScenarioIndex scenario, const int &nD, const double &thin, ob::ScopedState<> &start, ob::ScopedState<> &goal);

#endif	/* READENVIRONMENT_H */

