/*
 * BenchmarkSettings.h
 *
 *  Created on: Sep 2, 2014
 *      Author: jingru
 */

#ifndef BENCHMARKSETTINGS_H_
#define BENCHMARKSETTINGS_H_

#include <string>
#include "EnvironmentSettings.h"
#include "PlannerSettings.h"
#include "utility.h"

/** maximum time of seconds allowed for each planner to run */
static const double cutoff_seconds_2D = 60;
static const double cutoff_seconds_3D = 60;
static const double cutoff_seconds_HD = 60;

/** number of runs for each planner */
static const int nRuns = 1;

/** number of dimensions to test, i.e., size of array nD_to_test  */
static const int num_nD_to_test = 1;
/** array of dimensions to test  */
static const int nD_to_test[] = {2};

/** number of narrow passage width to test, i.e., size of array width_to_test  */
static const int num_width_to_test = 1;
/** array of width to test  */
static const double width_to_test[] = {0.2};

/** number of narrow passage length(thickness) to test, i.e., size of array thin_to_test  */
static const int num_thin_to_test = 1;
/** array of length(thickness) to test  */
static const double thin_to_test[] = {0.2};

/** number of problems (kink, 1-NP-class, 2-NP-class, planary linkage) to test  */
static const int num_scenarios_to_test = 1;
/** array of problems (	NarrowPassage_2_homotopy, NarrowPassage_1_homotopy,
 * PlanaryLinkage, NarrowKink_1_homotopy) to test
 */
static const ScenarioIndex scenarios_to_test[] = {NarrowPassage_2_homotopy};

/** set to true if a planner is intended to be benchmarked */
static const bool runMRRT = true;
static const bool runMRRT_shortcut = true;
static const bool runRRTStar = true;
static const bool runPRMStar = true;
static const bool runPRMStar_Lazy = true;
static const bool runCE = true;
static const bool runFMM = true;
static const bool runFMMOffset = true;

/** record best path in each iteration */
static const bool recordPath = true;

/** record planner data for planners from OMPL */
static const bool recordPlannerData = false;

static const long FMM_NUM_LIMIT = 10e+09;

/** folder name for store benchmark results for each planner  */
static const std::string MRRTFolder = "MRRT";
static const std::string MRRTFolder_shortcut = "MRRT_S";
static const std::string RRTStarFolder = "RRTStar";
static const std::string CEFolder = "CrossEntropy";
static const std::string PRMStarFolder = "PRMStar";
static const std::string PRMStarFolder_Lazy = "PRMStar_Lazy";
static const std::string FMMFolder = "FMM";
static const std::string FMMPerturbationFolder = "FMMPerturbation";
static const std::string FMMOffsetFolder = "FMMOffset";

/** write benchmark setting information to summary file in folder experimentDir */
void writeExperimentInfo(char *experimentDir, ScenarioIndex scenarioIndex);

/** create folders for storing benchmark results */
bool createTopDirs(char *experimentDir);


#endif /* BENCHMARKSETTINGS_H_ */
